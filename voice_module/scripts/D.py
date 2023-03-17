#!/usr/bin/env python
# encoding: utf-8

from re import T
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

        self.x = 0.08

        # 阈值
        self.thres=40
        self.abs_thres=50


        self.lidar_left = 0.5
        self.lidar_right = 0.5
        self.best = 40


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

    def waterCallBack(self, msg):
        if msg.data==1:
            self.water=1
            self.num+=1
            rospy.loginfo('self.num:%d',self.num)
        else:
            rospy.loginfo('Failed')

    def water_Subscriber(self):
        # rospy.Subscriber("/water", Int32, self.waterCallBack, queue_size=10)
        rospy.Subscriber("/judge", Int32, self.waterCallBack, queue_size=10)
    
    def LidarCallback(self,msg):
        if msg.ranges[350] != float('inf') and msg.ranges[350]<6.00:
            self.lidar_left = msg.ranges[350]*100
        if msg.ranges[950] != float('inf') and msg.ranges[950]<6.00:
            self.lidar_right = msg.ranges[950]*100
        # if self.i % 10 == 0:
            # rospy.loginfo("left1:[%f]",self.distance_left1)
            # rospy.loginfo("right1:[%f]",self.distance_right1)


    
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

            
    def distCallback(self,msg):
        global vel_pub,arrive_pub
        vel = Twist()
        self.distance_left1 = msg.left1
        self.distance_left2 = msg.left2
        self.distance_right1 = msg.right1
        self.distance_right2 = msg.right2
        self.test_flowerpot()
        if self.water==0:
            # 到达右边
            if self.right==1 and self.left==0:
                self.water_flag = 2
                # 开始平移
                self.water = 3

            # 到达左边
            elif self.right==0 and self.left==1:

                self.water_flag = 1
                # 开始平移
                self.water = 3
            elif self.left==1 and self.right==1:
                vel.linear.x = self.x
            elif self.left==0 and self.right==0:
                vel.linear.x = self.x

        # 刚浇完水
        elif self.water==1:
            if self.water_flag==2:             
                if self.distance_right1>self.abs_thres:
                    self.water=0
                else:
                    vel.linear.x = self.x
            elif self.water_flag==1:
                if self.distance_left1>self.abs_thres:
                    self.water=0
                else:
                    vel.linear.x = self.x
            else:
                vel.linear.x = self.x

        elif self.water == 2:
            if self.water_flag == 1:
                arrive_pub.publish(1)
                self.water = 10
            elif self.water_flag == 2:
                arrive_pub.publish(2)
                self.water = 10
            if self.water == 10:
                vel.linear.x = 0.0

        # 平移
        elif self.water == 3:

            # 如果到达右边
            if self.water_flag == 2:
                if self.lidar_right > 60:
                    vel.linear.x = -self.x
                else:
                    # 30 - 40 = -10
                    vel.linear.y = -(self.lidar_right - self.best)*0.01
                    rospy.loginfo("右边距离和理想值的差:[%.1f]",self.lidar_right - self.best)
                if abs(self.lidar_right - self.best)<1:
                    self.water = 2
            elif self.water_flag == 1:
                if self.lidar_left > 60:
                    vel.linear.x = -self.x
                else:
                    # 30 - 40 = -10
                    vel.linear.y = (self.lidar_left - self.best)*0.01
                    rospy.loginfo("左边距离和理想值的差:[%.1f]",self.lidar_left - self.best)
                if abs(self.lidar_left - self.best)<1:
                    self.water = 2
        rospy.loginfo("浇水位状态:[%d],花盆左右状态:[%d]",self.water,self.water_flag)
        vel_pub.publish(vel)
       
               

if __name__ == '__main__':
    voicer = Voicer()
    rospy.init_node("D",anonymous=True)
    rospy.Subscriber("/scan",LaserScan,voicer.LidarCallback,queue_size=10)
    rospy.Subscriber("/distance",DistanceMsg,voicer.distCallback,queue_size=10)
    rospy.Subscriber("/judge",Int32,voicer.waterCallBack,queue_size=10)
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    arrive_pub = rospy.Publisher("/arrive_info",Int32,queue_size=10)
    rospy.spin()
