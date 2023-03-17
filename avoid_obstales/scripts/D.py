#!/usr/bin/env python
# encoding: utf-8

import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Int32,Float32
from geometry_msgs.msg import Twist
from voice_module.msg import DistanceMsg
from threading import Thread
from sensor_msgs.msg import LaserScan
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

        self.l1,self.l2,self.r1,self.r2=0,0,0,0
        self.left,self.right=0,0
        
        self.flag = 1
        self.num = 0
        self.i = 0
        self.water = 0

        # 记录停车位置 左1/右2
        self.water_flag=0

        self.vel = 0.08

        # 阈值
        self.thres=40
        self.abs_thres=50
 
        self.Pl1,self.Pl2,self.Pr1,self.Pr2=1,1,1,1
        self.P_l1,self.P_l2,self.P_r1,self.P_r2=0,0,0,0
        self.Xl1,self.Xl2,self.Xr1,self.Xr2=0,0,0,0
        self.X_l1,self.X_l2,self.X_r1,self.X_r2=0,0,0,0
        self.Kl1,self.Kl2,self.Kr1,self.Kr2=0,0,0,0


        self.lidar_left = 0.5
        self.lidar_right = 0.5



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
        dis_pub.pub(self)
    def thread_job(self):
        try:
            rospy.spin()
            print('thread1-rospy.spin')
        except rospy.ROSInterruptException:
            print('exception_subscribe')

    def waterCallBack(self, msg):
        if msg.data==1:
            self.water=1
            self.num+=1
            rospy.loginfo('self.num:%d',self.num)
        else:
            rospy.loginfo('Failed')

    def water_Subscriber(self):
        rospy.Subscriber("/water", Int32, self.waterCallBack, queue_size=10)
        # rospy.Subscriber("/judge", Int32, self.waterCallBack, queue_size=10)
    
    def LidarCallback(self,msg):
        if msg.ranges[350] != float('inf') and msg.ranges[350]<3.00:
            self.lidar_left = msg.ranges[350]*100
        if msg.ranges[925] != float('inf') and msg.ranges[925]<3.00:
            self.lidar_right = msg.ranges[925]*100
        if self.i % 10 == 0:
            rospy.loginfo("left1:[%f]",self.lidar_left)
            rospy.loginfo("right1:[%f]",self.lidar_right)
    
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
        self.distance_left1=self.KLM(self.ultrasonic(self.TRIG_left1,self.ECHO_left1),"l1")
        self.distance_left2=self.KLM(self.ultrasonic(self.TRIG_left2,self.ECHO_left2),"l2")
        self.distance_right1=self.KLM(self.ultrasonic(self.TRIG_right1,self.ECHO_right1),"r1")
        self.distance_right2=self.KLM(self.ultrasonic(self.TRIG_right2,self.ECHO_right2),"r2")

    
    def test_obs(self,dis):
        if dis <= self.thres:
            return 1
        else:
            return 0

    def test_flowerpot(self):
        self.l1=self.test_obs(self.distance_left1)
        self.l2=self.test_obs(self.distance_left2)
        self.r1=self.test_obs(self.distance_right1)
        self.r2=self.test_obs(self.distance_right2)
        if self.l1==1 and self.l2==1:
            self.left=1
        else:
            self.left=0
        if self.r1==1 and self.r2==1:
            self.right=1
        else:
            self.right=0

    def arrive(self,direct):
        if direct == 2:
            self.water_flag = 2
            self.pub_distance(self.lidar_right)
        elif direct == 1:
            self.water_flag = 1
            self.pub_distance(self.lidar_left)
        self.water = 2 


    def move(self):
        self.get_distance()
        self.test_flowerpot()
        if self.water==0:
            # 到达右边
            if self.right==1 and self.left==0:
                self.arrive(2)
            # 到达左边
            elif self.right==0 and self.left==1:
                self.arrive(1)
            elif self.left==1 and self.right==1:
                self.pub_velocity(self.vel,0.0)
            elif self.left==0 and self.right==0:
                self.pub_velocity(self.vel,0.0)
        elif self.water==1:
            if self.water_flag==2:             
                if self.distance_right1>self.abs_thres:
                    self.water=0
                else:
                    self.pub_velocity(self.vel,0.0)
            elif self.water_flag==1:
                if self.distance_left1>self.abs_thres:
                    self.water=0
                else:
                    self.pub_velocity(self.vel,0.0)
            else:
                self.pub_velocity(self.vel,0.0)
        elif self.water == 2:
            self.pub_velocity(0.0,0.0)
            self.pub_isArrive(self.water_flag)
    
    def execute_func(self):
        while not rospy.is_shutdown():
            self.i += 1
            self.move()
            time.sleep(0.05)
            if self.num == 6:
                break

            
       
               

if __name__ == '__main__':
    voicer = Voicer()
    rospy.init_node('D', anonymous=True)
    voicer.water_Subscriber()
    rospy.Subscriber("/scan",LaserScan,voicer.LidarCallback,queue_size=10)

    thread1 = Thread(target=voicer.thread_job)
    thread2 = Thread(target=voicer.execute_func)

    thread1.start()
    thread2.start()
