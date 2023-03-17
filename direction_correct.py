#!/usr/bin/env python
#coding=utf-8

import rospy
import time
from std_msgs.msg import Int32, Float64
from geometry_msgs.msg import Twist
import threading

class Direction_Corrector():
    def __init__(self):
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.odom_subscriber = rospy.Subscriber('/odom', /odom, self.odom_callback)
        self.direction_start = Float64()
        
    def get_odom(self):
        
        
    def get_direction_start(self):
        rate = rospy.Rate(10)
        i = 0
        while( i < 10):
            
        
if __name__ == бо__main__':
    rospy.init_node('direction_correct')
    direction_corrector = Direction_Corrector()