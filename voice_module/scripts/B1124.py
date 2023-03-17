#!/usr/bin/env python
# encoding: utf-8

from datetime import timedelta
from re import T
from numpy import around
import rospy
import time
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist,Vector3
# from voice_module.msg import DistanceMsg
from threading import Thread
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import *

class Test():
    def __init__(self):
        self.cmd_vel_msg = Twist()
        self.Kp_dir = 0.5
        self.Ki_dir = 0.0
        self.Kd_dir = 0.0
        self.delta_ang = 0.0
        self.delta_ang_last = 0.0
        self.SumErrAng = 0.0
        self.count_euler = 0
        self.init_angel = 0.0
        self.direction_initfinish_flag = 0
        self.euler_angle_subscriber = rospy.Subscriber('/euler_angles', Vector3, self.euler_angle_callback)

        rospy.Subscriber("/scan",LaserScan,self.LidarCallback,queue_size=10)
        #rospy.Subscriber("/imu_nine",Imu,self.imu_callback,queue_size=10)
        rospy.Subscriber("/judge",Int32,self.waterCallback,queue_size=10)
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.arrive_pub = rospy.Publisher("/arrive_info",Int32,queue_size=10)

        self.cmd_vel_msg = Twist()
        self.sum_left_right = 120 #是否到达花盆的阈值
        self.dist_left = 80 #雷达量测距离
        self.dist_right = 80 #雷达量测距离
        self.ifwatering = 0 #是否正在浇水的标志位，发出到达信号后置1，收到water信号后置0
        self.ifnext = 1 #是否经过缝隙
        self.ifpubarrive = 0 #是否发送过达到信号
        self.watertime = time.time() #收到浇水完毕信号的时间
        self.euler_y = 0.0 #前进方向的欧拉角
        self.ifpingyi = 0 #是否左右平移调整过
        self.pingyi_left = 0.0
        self.pingyi_right = 0.0 #用于平移距离的数据记录
        self.flower_count = 0 #当前是第几个花盆，收到一次water+1
        self.detect_end_flag = 0 #是否开启检查B区结束位置，在浇完第三盆花后开启
        self.time_detect_last = time.time()
        self.sum_end = 450

        self.exec_flag = 0

    def euler_angle_callback(self, msg):
        if(self.exec_flag == 0):
            self.euler_y = msg.y
            self.cmd_vel_msg.angular.z = 0.0
            # print("reveive euler_angle_data")
            if(self.count_euler < 200):
                self.init_angel += msg.z
                self.count_euler += 1
                self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            else:
                if(self.count_euler == 200):
                    self.init_angel = self.init_angel / 200
                    self.count_euler += 1
                    self.direction_initfinish_flag = 1
                else:
                    self.delta_ang = - self.init_angel + msg.z
                    self.SumErrAng = self.SumErrAng + self.delta_ang
                    self.delta_ang_last = self.delta_ang
                    
                    if(abs(self.delta_ang) >= 0.03):
                        self.cmd_vel_msg.angular.z = self.Kp_dir * self.delta_ang + self.Ki_dir * self.SumErrAng + self.Kd_dir * (self.delta_ang - self.delta_ang_last)
                        if(abs(self.cmd_vel_msg.angular.z) >= 0.2):
                            if(self.cmd_vel_msg.angular.z > 0):
                                self.cmd_vel_msg.angular.z = 0.2
                            else:
                                self.cmd_vel_msg.angular.z = -0.2
    
    def LidarCallback(self, msg):
        if(self.exec_flag == 0):
            if(msg.ranges[350] < 6.00):
                self.dist_left = msg.ranges[350] * 100
            if(msg.ranges[950] < 6.00):
                self.dist_right = msg.ranges[950] * 100
            print("lidar_left:", self.dist_left, "----lidar_right:", self.dist_right)
    
    def waterCallback(self, msg):
        if(self.exec_flag == 0):
            if msg.data == 1:
                self.ifnext = 0
                #self.ifpubarrive = 0
                self.ifpingyi = 0
                self.flower_count += 1
                if(self.flower_count > 3):
                    self.detect_end_flag = 1
    
    def delete(self):
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`~~~~~~~~~~~~delete self")
        self.exec_flag = 1

    def execute_func(self):
        while(self.direction_initfinish_flag == 0):
            pass
        print("direction_init finished")
        while not rospy.is_shutdown():
            if(self.detect_end_flag == 1):
                if((self.dist_left + self.dist_right) > self.sum_end):
                    if((time.time() - self.time_detect_last) > 7):
                        self.time_detect_last  = time.time()
                    elif((time.time() - self.time_detect_last) > 3 and (time.time() - self.time_detect_last) < 7):
                        self.cmd_vel_msg.linear.x = 0
                        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                        self.delete()
                        break
                    else:
                        pass
                else:
                    pass

            if(self.ifnext == 1):
                if(time.time() - self.watertime > 3):
                    if((self.dist_left + self.dist_right) < self.sum_left_right):
                        # if(self.euler_y > 0.018):
                        #     self.cmd_vel_msg.linear.x = 0.1
                        #     self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                        # elif(self.euler_y < -0.018):
                        #     self.cmd_vel_msg.linear.x = -0.1
                        #     self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                        # else:
                        if(False):
                            self.cmd_vel_msg.linear.x = 0
                            if(self.dist_left - self.dist_right > 3):
                                if(self.ifpingyi == 0):
                                    time1 = time.time()
                                    while((time.time() - time1) < ((self.dist_left - self.dist_right)/2)):
                                        self.cmd_vel_msg.linear.y = 0.02
                                        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                                    self.ifpingyi = 1
                                    self.cmd_vel_msg.linear.y = 0.0
                            elif(self.dist_right - self.dist_left > 3):
                                if(self.ifpingyi == 0):
                                    time1 = time.time()
                                    while((time.time() - time1) < ((self.dist_right - self.dist_left)/2)):
                                        self.cmd_vel_msg.linear.y = -0.02
                                        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                                    self.ifpingyi = 1
                                    self.cmd_vel_msg.linear.y = 0.0
                            if(self.ifpubarrive == 0):
                                self.arrive_pub.publish(1)
                                self.ifpubarrive = 1
                        self.cmd_vel_msg.linear.x = 0
                        if(self.dist_left - self.dist_right > 5):
                            time1 = time.time()
                            while((time.time() - time1) < ((self.dist_left - self.dist_right)/2)):
                                self.cmd_vel_msg.linear.y = 0.02
                                self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                                if(self.dist_left + self.dist_right > self.sum_left_right):
                                    break
                            self.cmd_vel_msg.linear.y = 0.0
                        elif(self.dist_right - self.dist_left > 5):
                            time1 = time.time()
                            while((time.time() - time1) < ((self.dist_right - self.dist_left)/2)):
                                self.cmd_vel_msg.linear.y = -0.02
                                self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                                if(self.dist_left + self.dist_right > self.sum_left_right):
                                    break
                            self.cmd_vel_msg.linear.y = 0.0
                        if(self.ifpubarrive == 0):
                            self.arrive_pub.publish(1)
                            self.ifpubarrive = 1
                    else:
                        self.cmd_vel_msg.linear.x = 0.12
                        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.linear.x = 0.12
                    self.cmd_vel_publisher.publish(self.cmd_vel_msg)
            else:
                self.cmd_vel_msg.linear.x = 0.12
                self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                if((self.dist_left + self.dist_right) > self.sum_left_right):
                    self.ifnext = 1
                    self.ifpubarrive = 0
#     def thread_job(self):
#         try:
#             rospy.spin()
#             print('thread1-rospy.spin')
#         except rospy.ROSInterruptException:
#             print('exception_subscribe')

# if __name__ == '__main__':
#     rospy.init_node('B', anonymous=True)
#     test = Test()
#     thread1 = Thread(target=test.thread_job)
#     thread2 = Thread(target=test.execute_func)

#     thread1.start()
#     thread2.start()
