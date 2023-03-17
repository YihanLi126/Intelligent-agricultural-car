#!/usr/bin/env python
# encoding: utf-8

import rospy
import time
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist,Vector3
# from voice_module.msg import DistanceMsg
from threading import Thread
from sensor_msgs.msg import LaserScan

cmd_vel_msg = Twist()
cmd_vel_msg.linear.z = 0
cmd_vel_msg.angular.x = 0
cmd_vel_msg.angular.y = 0
cmd_vel_msg.linear.x = 0
cmd_vel_msg.linear.y = 0
cmd_vel_msg.angular.z = 0
direction_initfinish_flag = 0

class Direction_Corrector():
    def __init__(self):
        self.Kp_dir = 0.5
        self.Ki_dir = 0.0
        self.Kd_dir = 0.0
        self.delta_ang = 0.0
        self.delta_ang_last = 0.0
        self.SumErrAng = 0.0
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.count_euler = 0
        self.init_angel = 0.0
        self.euler_angle_subscriber = rospy.Subscriber('/euler_angles', Vector3, self.euler_angle_callback)

    def euler_angle_callback(self, msg):
        cmd_vel_msg.angular.z = 0.0
        print("reveive euler_angle_data")
        if(self.count_euler < 200):
            self.init_angel += msg.z
            self.count_euler += 1
            print("count_euler = ", self.count_euler)
            self.cmd_vel_publisher.publish(cmd_vel_msg)
        else:
            if(self.count_euler == 200):
                self.init_angel = self.init_angel / 200
                self.count_euler += 1
                print("count_euler = ", self.count_euler)
                global direction_initfinish_flag
                direction_initfinish_flag = 1
            else:
                print("count_euler = ", self.count_euler)
                self.delta_ang = - self.init_angel + msg.z
                self.SumErrAng = self.SumErrAng + self.delta_ang
                self.delta_ang_last = self.delta_ang
                
                if(abs(self.delta_ang) >= 0.03):
                    cmd_vel_msg.angular.z = self.Kp_dir * self.delta_ang + self.Ki_dir * self.SumErrAng + self.Kd_dir * (self.delta_ang - self.delta_ang_last)
                    if(abs(cmd_vel_msg.angular.z) >= 0.2):
                        if(cmd_vel_msg.angular.z > 0):
                            cmd_vel_msg.angular.z = 0.2
                        else:
                            cmd_vel_msg.angular.z = -0.2

class Voicer:
    def __init__(self):

        # 距离变量
        self.distance_left1 = 50

        self.water_time = []
        self.front_flag = 0
        self.back_leave = 1
        
        self.flag = 1
        self.num = 0
        self.i = 0
        self.status = 0
        self.water=0

        # 左轮阈值
        self.left1_thres = 45

        # linear.x值
        self.x = 0.08

    def pub_isArrive(self,flag):
        arrive_pub=rospy.Publisher('/arrive_info',Int32,queue_size=10)
        dis_pub=rospy.Publisher('/us_data', Float32, queue_size=10)
        dis_pub.publish(self.distance_left1*100)
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
        # turtle_vel_pub.publish(vel_msg)
        
        if self.i%10 == 0:
            rospy.loginfo("Publsh velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)

    def LidarCallback(self, msg):
        if msg.ranges[350] != float('inf') and msg.ranges[350]<3.00:
            self.distance_left1 = msg.ranges[350]*100
            rospy.loginfo("distance:[%f]",self.distance_left1)
            rospy.loginfo("left_flag:[%d]",self.front_flag)
        
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
            self.pub_isArrive(0)
            self.water_time.append(time.time())
            # rospy.loginfo(self.back_leave)
            rospy.loginfo('self.num:%d',self.num)
        else:
            rospy.loginfo('Failed')

    def water_Subscriber(self):
        # rospy.Subscriber("/water", Int32, self.waterCallBack, queue_size=10)
        rospy.Subscriber("/judge", Int32, self.waterCallBack, queue_size=10)

    
    def test_obs(self,left):
        if left <= self.left1_thres:
            return 1
        else: 
            return 0

    def move(self):
        self.front_flag = self.test_obs(self.distance_left1) 
        rospy.loginfo("water:[%d],front_flag:[%d],len(water_time):[%d]",self.water,self.front_flag,
        len(self.water_time))
        if self.water==0:
            if self.front_flag==1:
                if len(self.water_time)==0:
                    self.water_time.append(time.time())
                    self.pub_velocity(0.0,cmd_vel_msg.angular.z)
                    # self.pub_isArrive(1)
                elif len(self.water_time) >= 2:
                    if time.time()-self.water_time[-1]<3:
                        self.pub_velocity(self.x,cmd_vel_msg.angular.z)
                        self.water=1
                    else:
                        self.water_time.append(time.time())
                        self.pub_velocity(0.0,cmd_vel_msg.angular.z)
                        rospy.loginfo("delta time:[%.0f]",self.water_time[-1]-self.water_time[-2])
                        self.water=2
                else:
                    self.water=2

            elif self.front_flag==0:
                # self.pub_isArrive(0)
                self.pub_velocity(self.x,cmd_vel_msg.angular.z)
        elif self.water==1:
            if self.front_flag==0:
                self.water=0
            else:
                # self.pub_isArrive(0)
                self.pub_velocity(self.x,cmd_vel_msg.angular.z)
        elif self.water==2:
            # self.pub_isArrive(1)
            self.pub_velocity(0.0,cmd_vel_msg.angular.z)
    


    def execute_func(self):
        direction_corrector = Direction_Corrector()
        global direction_initfinish_flag
        while(direction_initfinish_flag == 0):
          pass
        while not rospy.is_shutdown():
            self.i += 1
            self.move()
            time.sleep(0.02)
            if self.num == 6:
                break
            
       
               

if __name__ == '__main__':
    voicer = Voicer()
    rospy.init_node('A', anonymous=True)
    rospy.Subscriber("/scan",LaserScan,voicer.LidarCallback,queue_size=10)
    voicer.water_Subscriber()

    thread1 = Thread(target=voicer.thread_job)
    thread2 = Thread(target=voicer.execute_func)

    thread1.start()
    thread2.start()