#!/usr/bin/env python
#encoding: utf-8

import rospy
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from threading import Thread
from sensor_msgs.msg import LaserScan
from simple_pid import PID


class Adjuster:
    def __init__(self):
        self.min_angular = 0
        self.ang_error = 0
        self.ang_refer = 80 #正前方的角度，需要再确认
        self.ang_v = 0 #输出角速度

    def LidarCallback(self, msg):
    	#开始扫描角度，需要再确认
    	start = 74
    	end = 86
    	self.angular = start
    	for i in range(start, end+1):
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
