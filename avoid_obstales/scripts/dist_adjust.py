#!/usr/bin/env python
#encoding: utf-8

import rospy
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from threading import Thread
from sensor_msgs.msg import LaserScan

cmd_vel_msg = Twist()
cmd_vel_msg.linear.z = 0
cmd_vel_msg.angular.x = 0
cmd_vel_msg.angular.y = 0
cmd_vel_msg.linear.x = 0
cmd_vel_msg.linear.y = 0


class PID:
    def __init__(self):
        self.Kp = rospy.get_param('/Kp_dir_param')
        self.Ki = rospy.get_param('/Ki_dir_param')
        self.Kd = rospy.get_param('/Kd_dir_param')

        self.sample_time = 0.00 #采样间隔
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0
        self.I_max_modify = 0.08 #积分项最大的波动值 
 
    def update(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
 
        if(delta_time >= self.sample_time):
            self.ITerm += error * delta_time
            if(self.ITerm < -self.I_max_modify):
                self.ITerm = -self.I_max_modify
            elif(self.ITerm > self.I_max_modify):
                self.ITerm = self.I_max_modify
 
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = (error - self.last_error) / delta_time
 
            self.last_time = self.current_time
            self.last_error = error
            self.output = (self.Kp * error) + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

class Adjuster:
    def __init__(self):
        self.dist = 0

    def LidarCallback(self, msg):
    	#开始扫描角度，需要再确认
    	start = 1150
    	end = 1250
        dist = 0
    	for i in range(start, end):
    		if msg.ranges[i] < msg.ranges[min_angular]:
    			self.min_angular = i
    			self.ang_error = self.min_angular - self.ang_refer

    def read_radar(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            print('exception_read_radar')

    def pub_angular(self):
    	#利用PID得到输出ang_v，输入是角度误差ang_error，目标为0
    	pid = PID(2, 0.01, 0.1, setpoint=0)
    	ang_pub = rospy.Publisher('/angular', Int32, queue_size=10)
        while not rospy.is_shutdown():
        	self.ang_v = pid(self.ang_error)
        	ang_pub.publish(self.ang_v)
        	



if __name__ == '__main__':
    adjuster = Adjuster()
    rospy.init_node('adjust', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, adjuster.LidarCallback, queue_size=10)

    thread1 = Thread(target=adjuster.read_radar)
    thread2 = Thread(target=adjuster.pub_angular)
    thread1.start()
    thread2.start()
