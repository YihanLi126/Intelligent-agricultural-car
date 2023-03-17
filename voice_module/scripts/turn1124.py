#! /usr/bin/env python
#coding=utf-8

from curses import KEY_DC
import rospy
import time
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3
from threading import Thread
from std_srvs.srv import Empty
import Queue
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan

class Corrector():
    def __init__(self, flag, rotate, length):
        #if move toward, flag = 0; if rotate, flag = 1
        #if rotate = 1, turn right; if rotate = 2, turn left; if rotate = 0, move toward
        #eg:直行corrector(0, 0, length);右转(1, 1, 0);左转(1, 2, 0)
        self.Kp_dir = 0.5
        self.Ki_dir = 0.0
        self.Kd_dir = 0.0
        self.flag = flag
        self.rotate = rotate
        print("in the init func: rotate = ", rotate)
        self.length = length
        self.laserscan_subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.euler_angle_subscriber = rospy.Subscriber('/euler_angles', Vector3, self.euler_angle_callback)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.length_init = 0.0
        self.length_goal = 0.0
        self.length_count = 0
        self.length_delta = 0.0
        self.length_delta_last = 0.0
        self.SumErrLen = 0.0
        self.ang_init = 0.0
        self.ang_goal = 0.0
        self.ang_count = 0
        self.ang_delta = 0.0
        self.ang_delta_last = 0.0
        self.SumErrAng = 0.0
        self.ang_flag = 0
        self.exec_flag = 0
        self.cmd_vel_msg = Twist()
        self.cmd_vel_msg.linear.z = 0
        self.cmd_vel_msg.angular.x = 0
        self.cmd_vel_msg.angular.y = 0
        self.cmd_vel_msg.linear.x = 0
        self.cmd_vel_msg.linear.y = 0
    def euler_angle_callback(self, msg):
        if(self.exec_flag == 0):
            print("received euler_angle")
            #only change self.cmd_vel_msg.angular.z
            self.cmd_vel_msg.angular.z = 0.0
            if(self.ang_count < 100):
                self.ang_init += msg.z
                self.ang_count += 1
            elif(self.ang_count == 100):
                self.ang_init = self.ang_init / 100
                self.ang_count += 1
                if(self.rotate == 0):
                    self.ang_goal = self.ang_init # move toward
                    print("rotate = 0, move forward, ang_goal = ", self.ang_goal)
                elif(self.rotate == 2):
                    self.ang_goal = self.ang_init - (3.1415927 / 2) # turn left
                    if(self.ang_goal < 0):
                        self.ang_goal = 3.1415926 * 2 + self.ang_goal
                    print("rotate = 2, turn left, ang_goal = ", self.ang_goal)
                else:
                    self.ang_goal = self.ang_init + (3.1415927 / 2) # turn right
                    if(self.ang_goal > 3.1415926 * 2):
                        self.ang_goal = self.ang_goal - 3.1415926 * 2
                    print("rotate = 1, turn right, ang_goal = ", self.ang_goal)
                self.ang_flag = 1
            else:
                if(self.rotate == 0):
                    i = 0
                    self.ang_delta = - self.ang_goal + msg.z
                    # if(self.ang_delta >= 3.1415926):
                    #     self.ang_delta = -self.ang_goal + msg.z - 3.1415926 * 2
                    # elif(self.ang_delta <= -3.1415926):
                    #     self.ang_delta = self.ang_goal + 3.1415926 * 2 - msg.z
                    self.SumErrAng += self.ang_delta
                    self.ang_delta_last = self.ang_delta
                    if(abs(self.ang_delta) >= 0.03):
                        self.cmd_vel_msg.angular.z = self.Kp_dir * self.ang_delta + self.Ki_dir * self.SumErrAng + self.Kd_dir * (self.ang_delta - self.ang_delta_last)
                        if(abs(self.cmd_vel_msg.angular.z) >= 0.2):
                            if(self.cmd_vel_msg.angular.z > 0):
                                self.cmd_vel_msg.angular.z = 0.2
                            else:
                                self.cmd_vel_msg.angular.z = -0.2
                    # if(self.flag == 1):
                    #     print("rotate publishing cmd_vel: ", self.cmd_vel_msg.angular.z)
                    #     self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                    #     if(abs(self.ang_delta) < 0.03):
                    #         self.delete()
                elif(self.rotate == 1):
                    self.ang_delta = - self.ang_goal + msg.z
                    if(abs(self.ang_delta) >= 0.03):
                        self.cmd_vel_msg.angular.z = -0.2
                        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                        print("rotate 1 publishing cmd_vel: ", self.cmd_vel_msg.angular.z)
                    else:
                        self.cmd_vel_msg.angular.z = 0.0
                        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                        self.delete()
                else:
                    self.ang_delta = - self.ang_goal + msg.z
                    if(abs(self.ang_delta) >= 0.03):
                        self.cmd_vel_msg.angular.z = 0.2
                        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                        print("rotate 2 publishing cmd_vel: ", self.cmd_vel_msg.angular.z)
                    else:
                        self.cmd_vel_msg.angular.z = 0.0
                        self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                        self.delete()
                
                
    def laserscan_callback(self, msg):
        if(self.exec_flag == 0):
            print("received laserscan")
            while(self.ang_flag == 0):
                pass
            if(self.flag == 0):
                if(self.length_count < 50):
                    self.length_init += msg.ranges[0]
                    self.length_count += 1
                    self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                else:
                    if(self.length_count == 50):
                        self.length_init = self.length_init / 50
                        self.length_count += 1
                        self.length_goal = self.length_init - self.length
                    else:
                        self.length_delta = msg.ranges[0] - self.length_goal
                        self.SumErrLen += self.length_delta
                        self.length_delta_last = self.length_delta
                        if(abs(self.length_delta) >= 0.01):
                            self.cmd_vel_msg.linear.x = self.Kp_dir * self.length_delta_last + self.Ki_dir * self.SumErrLen + self.Kd_dir * (self.length_delta - self.length_delta_last)
                            if(abs(self.cmd_vel_msg.linear.x) >= 0.08):
                                if(self.cmd_vel_msg.linear.x > 0):
                                    self.cmd_vel_msg.linear.x = 0.08
                                else:
                                    self.cmd_vel_msg.linear.x = -0.08
                                print("move forward publishing cmd_vel: ", self.cmd_vel_msg.linear.x)
                                self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                        else:
                            self.cmd_vel_msg.linear.x = 0.0
                            self.cmd_vel_publisher.publish(self.cmd_vel_msg)
                            self.delete()
                        
    def delete(self):
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`~~~~~~~~~~~~delete self")
        self.exec_flag = 1

    def execute_func(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            if(self.exec_flag == 1):
                break
            r.sleep()
# if __name__ == '__main__':
#     rospy.init_node('t')
#     t = Corrector(0, 0, 0.6)
#     t.execute_func()
#     rospy.spin()
