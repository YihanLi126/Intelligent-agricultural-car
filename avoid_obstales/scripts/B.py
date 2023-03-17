#!/usr/bin/env python
# encoding: utf-8

import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Int32,Float32
from geometry_msgs.msg import Twist
from voice_module.msg import DistanceMsg
from threading import Thread
GPIO.setmode(GPIO.BCM)

class Voicer:
    def __init__(self):
        self.space_between = 0.15
        self.TRIG_left1 = 12
        self.ECHO_left1 = 16
        self.TRIG_left2 = 4
        self.ECHO_left2 = 17
        self.TRIG_right1 = 27
        self.ECHO_right1 = 22
        self.TRIG_right2 = 9
        self.ECHO_right2 = 10

        # 距离变量
        self.distance_left1 = 0
        self.distance_left2 = 0
        self.distance_right1 = 0
        self.distance_right2 = 0

        self.starttime = time.time()
        self.endtime = time.time()

        self.front_flag = 0
        self.back_flag = 0
        self.back_leave = 1
        
        self.flag = 1
        self.num = 0
        self.i = 0
        self.status = 0

        self.water = 0

        # 左右轮阈值
        self.thres = 40
        # 左右轮和阈值
        self.abs_thres = 100

        self.Pl1,self.Pl2,self.Pr1,self.Pr2=1,1,1,1
        self.P_l1,self.P_l2,self.P_r1,self.P_r2=0,0,0,0
        self.Xl1,self.Xl2,self.Xr1,self.Xr2=0,0,0,0
        self.X_l1,self.X_l2,self.X_r1,self.X_r2=0,0,0,0
        self.Kl1,self.Kl2,self.Kr1,self.Kr2=0,0,0,0



        # 常量            
        self.Q=0.05 # 噪声
        self.R=0.5  # R如果很大，更相信预测值，那么传感器反应就会迟钝，反之相反


        GPIO.setup(self.TRIG_left1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ECHO_left1, GPIO.IN)
        GPIO.setup(self.TRIG_left2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ECHO_left2, GPIO.IN)
        GPIO.setup(self.TRIG_right1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ECHO_right1, GPIO.IN)
        GPIO.setup(self.TRIG_right2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ECHO_right2, GPIO.IN)

    # 卡尔曼滤波
    def KLM(self,Z,type):
        if type=="l1":
            self.X_l1 = self.Xl1+0
            self.P_l1 = self.Pl1+self.Q
            self.Kl1 = self.P_l1/(self.P_l1+self.R)
            self.Xl1 = self.X_l1+self.Kl1*(Z-self.X_l1)
            self.Pl1 = self.P_l1-self.Kl1*self.P_l1
            return self.Xl1
        if type=="l2":
            self.X_l2 = self.Xl2+0
            self.P_l2 = self.Pl2+self.Q
            self.Kl2 = self.P_l2/(self.P_l2+self.R)
            self.Xl2 = self.X_l2+self.Kl2*(Z-self.X_l2)
            self.Pl2 = self.P_l2-self.Kl2*self.P_l2
            return self.Xl2
        if type=="r1":
            self.X_r1 = self.Xr1+0
            self.P_r1 = self.Pr1+self.Q
            self.Kr1 = self.P_r1/(self.P_r1+self.R)
            self.Xr1 = self.X_r1+self.Kr1*(Z-self.X_r1)
            self.Pr1 = self.P_r1-self.Kr1*self.P_r1
            return self.Xr1
        if type=="r2":
            self.X_r2 = self.Xr2+0
            self.P_r2 = self.Pr2+self.Q
            self.Kr2 = self.P_r2/(self.P_r2+self.R)
            self.Xr2 = self.X_r2+self.Kr2*(Z-self.X_r2)
            self.Pr2 = self.P_r2-self.Kr2*self.P_r2
            return self.Xr2    

    def pub_isArrive(self,flag):
        arrive_pub=rospy.Publisher('/arrive_info',Int32,queue_size=10)
        arrive_pub.publish(flag)

    def pub_velocity(self,x, z):
               
        # 创建一个Publisher，发布名为/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
        turtle_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # 创建Twist()消息类型
        vel_msg = Twist()
        vel_msg.angular.z = z
        vel_msg.linear.x = x
        
        # while True:
        # 发布速度命令
        turtle_vel_pub.publish(vel_msg)
        
        if self.i%10 == 0:
            rospy.loginfo("Publsh velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)

    # 发布距离
    def pub_distance(self):
        dis_pub=rospy.Publisher('/us_data', Float32, queue_size=10)
        dis_pub.publish(self.distance_left1)

        front_pub=rospy.Publisher('/front',Int32,queue_size=10)
        back_pub=rospy.Publisher("/back",Int32,queue_size=10)
        self.get_distance()
        # dist_msg = DistanceMsg()
        # dist_msg.left1 = self.distance_left1
        # dist_msg.left1=self.distance_left1+self.distance_right1
        # dist_msg.right1 = self.distance_left2+self.distance_right2
        # dist_msg.left2 = self.distance_left2
        # dist_msg.right2 = self.distance_right2
        # dist_msg.left2,dist_msg.right2=0,0
        back_pub.publish(self.back_flag)
        front_pub.publish(self.front_flag)
        # rospy.loginfo("left1[%.0f],right1[%.0f],left2[%.0f],right2[%.0f]", dist_msg.left1,
        # dist_msg.right1,dist_msg.left2,dist_msg.right2)
        rospy.loginfo("front:[%d],back:[%d]",self.front_flag,self.back_flag)
        
    def thread_job(self):
        try:
            rospy.spin()
            print('thread1-rospy.spin')
        except rospy.ROSInterruptException:
            print('exception_subscribe')

    def waterCallBack(self, msg):
        if msg.data==1:
            self.back_leave=0
            self.num+=1
            rospy.loginfo(self.back_leave)
            rospy.loginfo('self.num:%d',self.num)
        else:
            rospy.loginfo('Failed')

    def water_Subscriber(self):
        rospy.Subscriber("/judge", Int32, self.waterCallBack, queue_size=10)
    
    # 测距函数
    def ultrasonic(self,TRIG,ECHO):
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
        self.starttime=time.time()
        while GPIO.input(ECHO)==1:
            pass
        self.endtime=time.time()
        if(error_flag == 0):
            distance=round((self.endtime - self.starttime) * 17150, 2) 
            return distance
        else:
            return 0
            
    
    def get_distance(self):
        left2_origin = self.ultrasonic(self.TRIG_left2,self.ECHO_left2)
        self.distance_left1=self.KLM(self.ultrasonic(self.TRIG_left1,self.ECHO_left1),"l1")
        self.distance_left2=self.KLM(left2_origin,"l2")
        self.distance_right1=self.KLM(self.ultrasonic(self.TRIG_right1,self.ECHO_right1),"r1")
        self.distance_right2=self.KLM(self.ultrasonic(self.TRIG_right2,self.ECHO_right2),"r2")
        # rospy.loginfo("left1:[%f],right1:[%f]",self.distance_left1,self.distance_right1)
        # rospy.loginfo("left2:[%f],right2:[%f]",left2_origin,self.distance_right2)

    
    def test_obs(self,left,right):
        if left <= self.thres or right <= self.thres:
            if left + right <= self.abs_thres:
                return 1
            else:
                return 0
        else:
            return 0

    def move(self):
        self.get_distance()
        self.front_flag = self.test_obs(self.distance_left1,self.distance_right1)
        self.back_flag = self.test_obs(self.distance_left2,self.distance_right2)
        if self.front_flag==1:
            if self.back_leave==1:
                if self.back_flag==1:
                    self.pub_velocity(0.0,0.0)
                    self.back_leave = 2
                    # self.pub_isArrive(1)
                elif self.back_flag==0:
                    self.pub_velocity(0.12,0.0)
                    # self.pub_isArrive(0)
            elif self.back_leave==0:                                   
                if self.back_flag==1:
                    self.pub_velocity(0.12,0.0)
                    # self.pub_isArrive(0)
                elif self.back_flag==0:
                    self.pub_velocity(0.12,0.0)
                    self.back_leave=1
                    # self.pub_isArrive(0)
            elif self.back_leave == 2:
                self.pub_velocity(0.0,0.0) 
                
        elif self.front_flag==0:
            self.pub_velocity(0.12,0.0)
            # self.pub_isArrive(0)        
    


    def execute_func(self):
        while not rospy.is_shutdown():
            self.i += 1
            self.move()
            if self.i % 15 == 0:
                rospy.loginfo("left1:[%f],right1:[%f],left2:[%f],right2:[%f]",self.distance_left1,
                self.distance_right1,self.distance_left2,self.distance_right2)
                rospy.loginfo("front_flag:[%d],back_flag:[%d],back_leave:[%d]",
                self.front_flag,self.back_flag,self.back_leave)
            time.sleep(0.05)
            if self.num == 6:
                break

            
       
               

if __name__ == '__main__':
    voicer = Voicer()
    rospy.init_node('B', anonymous=True)
    voicer.water_Subscriber()


    thread1 = Thread(target=voicer.thread_job)
    thread2 = Thread(target=voicer.execute_func)

    thread1.start()
    thread2.start()
