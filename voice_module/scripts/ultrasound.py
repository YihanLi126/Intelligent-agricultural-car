#!/usr/bin/env python
# encoding: utf-8

import rospy
import RPi.GPIO as GPIO
import time
from voice_module.msg import DistanceMsg
GPIO.setmode(GPIO.BCM)
# 超声打开
TRIG_left1 = 12
ECHO_left1 = 16
TRIG_left2 = 4
ECHO_left2 = 17
TRIG_right1 = 27
ECHO_right1 = 22
TRIG_right2 = 9
ECHO_right2 = 10
# 超声距离
ult_left1 = 0
ult_left2 = 0
ult_right1 = 0
ult_right2 = 0
# 超声距离标志位
front_flag = 0
back_flag = 0
water = 0
# 超声循环次数
i = 0
# 卡尔曼滤波参数
Pl1,Pl2,Pr1,Pr2=1,1,1,1
P_l1,P_l2,P_r1,P_r2=0,0,0,0
Xl1,Xl2,Xr1,Xr2=0,0,0,0
X_l1,X_l2,X_r1,X_r2=0,0,0,0
Kl1,Kl2,Kr1,Kr2=0,0,0,0
# 常量            
Q=0.05 # 噪声
R=0.5  # R如果很大，更相信预测值，那么传感器反应就会迟钝，反之相反

GPIO.setup(TRIG_left1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO_left1, GPIO.IN)
GPIO.setup(TRIG_left2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO_left2, GPIO.IN)
GPIO.setup(TRIG_right1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO_right1, GPIO.IN)
GPIO.setup(TRIG_right2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO_right2, GPIO.IN)
# 超声距离publisher
ult_pub = rospy.Publisher("/distance",DistanceMsg,queue_size=10)
# 卡尔曼滤波
def KLM(Z,type):
    global Pl1,Pl2,Pr1,Pr2,P_l1,P_l2,P_r1,P_r2,Xl1,Xl2,Xr1,Xr2,X_l1,X_l2,X_r1,X_r2,Kl1,Kl2,Kr1,Kr2
    if type=="l1":
        X_l1 = Xl1+0
        P_l1 = Pl1+Q
        Kl1 = P_l1/(P_l1+R)
        Xl1 = X_l1+Kl1*(Z-X_l1)
        Pl1 = P_l1-Kl1*P_l1
        return Xl1
    if type=="l2":
        X_l2 = Xl2+0
        P_l2 = Pl2+Q
        Kl2 = P_l2/(P_l2+R)
        Xl2 = X_l2+Kl2*(Z-X_l2)
        Pl2 = P_l2-Kl2*P_l2
        return Xl2
    if type=="r1":
        X_r1 = Xr1+0
        P_r1 = Pr1+Q
        Kr1 = P_r1/(P_r1+R)
        Xr1 = X_r1+Kr1*(Z-X_r1)
        Pr1 = P_r1-Kr1*P_r1
        return Xr1
    if type=="r2":
        X_r2 = Xr2+0
        P_r2 = Pr2+Q
        Kr2 = P_r2/(P_r2+R)
        Xr2 = X_r2+Kr2*(Z-X_r2)
        Pr2 = P_r2-Kr2*P_r2
        return Xr2

# 测距函数
def ultrasonic(TRIG,ECHO):
    GPIO.output(TRIG,True)
    time.sleep(0.00001)
    GPIO.output(TRIG,False)
    i = 0
    error_flag = 0
    while GPIO.input(ECHO)==0:
        i = i + 1
        if(i > 5000):
            error_flag = 1
            break
        pass
    starttime=time.time()
    while GPIO.input(ECHO)==1:
        pass
    endtime=time.time()
    if(error_flag == 0):
        distance=round((endtime - starttime) * 17150, 2) 
        return distance
    else:
        return 0

def get_distance():
    global ult_left1,ult_left2,ult_right1,ult_right2,ult_pub
    ult_left1=KLM(ultrasonic(TRIG_left1,ECHO_left1),"l1")
    ult_left2=KLM(ultrasonic(TRIG_left2,ECHO_left2),"l2")
    ult_right1=KLM(ultrasonic(TRIG_right1,ECHO_right1),"r1")
    ult_right2=KLM(ultrasonic(TRIG_right2,ECHO_right2),"r2")

    ult = DistanceMsg()
    ult.left1 = ult_left1
    ult.left2 = ult_left2
    ult.right1 = ult_right1
    ult.right2 = ult_right2
    ult_pub.publish(ult)
rospy.init_node("ultrasound")
while True:
	get_distance()
GPIO.cleanup()
	