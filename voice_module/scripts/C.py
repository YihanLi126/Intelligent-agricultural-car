#!/usr/bin/env python3
# coding=utf-8
import rospy
from sensor_msgs.msg import LaserScan

#!/usr/bin/env python
# encoding: utf-8

import rospy
import time
from std_msgs.msg import Int32,Float32
from geometry_msgs.msg import Twist
from threading import Thread
from sensor_msgs.msg import LaserScan

class Voicer:
    def __init__(self):
        # 距离变量
        self.distance_left1 = 0.55
        self.distance_right1 = 0.55

        self.l1 =0
        self.r1 = 0  
        
        self.flag = 1
        self.num = 0
        self.i = 0
        self.water = 0

        self.vel=0.08

        # 记录停车位置 左1/右2
        self.water_flag=0

        # 阈值
        self.thres=0.50    

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
        
        # if self.i%10 == 0:
        #     rospy.loginfo("Publsh velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)

    # 发布距离
    def pub(self,dist):
        
        dis_pub=rospy.Publisher('/us_data', Float32, queue_size=10)
        # water_pub=rospy.Publisher("/water_flag",Int32,queue_size=10)
        # left_or_right_pub = rospy.Publisher("/left_or_right",Int32,queue_size=10)        
        dis_pub.publish(dist)
        # water_pub.publish(self.water)
        # left_or_right_pub.publish(self.water_flag)
        
        
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

    def LidarCallback(self, msg):
        if msg.ranges[350] != float('inf') and msg.ranges[350]<3.00:
            self.distance_left1 = msg.ranges[350]
        if msg.ranges[925] != float('inf') and msg.ranges[925]<3.00:
            self.distance_right1 = msg.ranges[925]
        if self.i % 10 == 0:
            rospy.loginfo("left1:[%f]",self.distance_left1)
            rospy.loginfo("right1:[%f]",self.distance_right1)

    def water_Subscriber(self):
        rospy.Subscriber("/water", Int32, self.waterCallBack, queue_size=10)
        # rospy.Subscriber("/judge", Int32, self.waterCallBack, queue_size=10)

    
    def test_obs(self,dis):
        if dis <= self.thres:
            return 1
        else:
            return 0

    def test_flowerpot(self):
        self.l1=self.test_obs(self.distance_left1)
        self.r1=self.test_obs(self.distance_right1)

    def move(self):
        # 未浇水状态
        if self.water==0:
            # 到达右边
            if self.r1==1 and self.l1==0:
                self.water_flag=2
                self.water = 2
                self.pub(self.distance_right1*100)
                # self.pub_isArrive(2)
            # 到达左边
            elif self.r1==0 and self.l1==1:
                self.water_flag=1
                self.water = 2 
                self.pub(self.distance_left1*100)
                # self.pub_isArrive(1)

            # 如果两个花盆都检测到了？
            # elif self.left==1 and self.right==1:
            #     self.pub_velocity(0.2,0.0)
            elif self.l1==0 and self.r1==0:
                self.pub_velocity(self.vel,0.0)
        # 浇完水状态
        elif self.water==1:
            # 如果刚刚在右边浇了水
            if self.water_flag==2:             
                if self.distance_right1>self.thres:
                    self.water=0
                else:
                    self.pub_velocity(self.vel,0.0)
            elif self.water_flag==1:
                if self.distance_left1>self.thres:
                    self.water=0
                else:
                    self.pub_velocity(self.vel,0.0)
            else:
                self.pub_velocity(self.vel,0.0)
        # 浇水状态
        elif self.water==2:
            self.pub_velocity(0.0, 0.0)
        
    
    def execute_func(self):
        while not rospy.is_shutdown():
            self.i += 1
            
            self.test_flowerpot()
            # self.pub()
            self.move()
            if self.num == 12:
                break
            # time.sleep(0.05)

            
       
               

if __name__ == '__main__':
    voicer = Voicer()
    rospy.init_node('C', anonymous=True)
    rospy.Subscriber("/scan",LaserScan,voicer.LidarCallback,queue_size=10)
    voicer.water_Subscriber()

    thread1 = Thread(target=voicer.thread_job)
    thread2 = Thread(target=voicer.execute_func)

    thread1.start()
    thread2.start()
