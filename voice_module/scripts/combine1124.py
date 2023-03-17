#! /usr/bin/env python
#coding=utf-8

import rospy
import time
import tf
from threading import Thread
from std_srvs.srv import Empty
from numpy import around
import RPi.GPIO as GPIO

from std_msgs.msg import Int32, Float64, Int64, Float32
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3
from sensor_msgs.msg import Imu, LaserScan
from voice_module.msg import DistanceMsg

from tf.transformations import euler_from_quaternion
from math import *

import A1124
import B1124
import turn1124
import D1124

def thread_job():
    try:
        rospy.spin()
        print('thread1-rospy.spin')
    except rospy.ROSInterruptException:
        print('exception_subscribe')

def execute_func():
    # plan_A = A1124.Test()
    # plan_A.execute_func()
    # plan_A_B_1 = turn1124.Corrector(0, 0, 0.73)#0.73
    # plan_A_B_1.execute_func()
    # print("plan_A_B_1 finished")
    # plan_A_B_2 = turn1124.Corrector(1, 1, 0)
    # plan_A_B_2.execute_func()
    # print("plan_A_B_2 finished")
    # plan_A_B_3 = turn1124.Corrector(0, 0, 1.10)
    # plan_A_B_3.execute_func()
    # plan_A_B_4 = turn1124.Corrector(1, 1, 0)
    # plan_A_B_4.execute_func()
    # plan_B = B1124.Test()
    # plan_B.execute_func()
    # # plan_B_D_1 = turn1124.Corrector(0, 0, 0.73)
    # # plan_B_D_1.execute_func()
    # plan_B_D_2 = turn1124.Corrector(1, 2, 0)
    # plan_B_D_2.execute_func()
    # plan_B_D_3 = turn1124.Corrector(0, 0, 2.42)
    # plan_B_D_3.execute_func()
    # plan_B_D_4 = turn1124.Corrector(1, 2, 0)
    # plan_B_D_4.execute_func()
    plan_D = D1124.Voicer()
    plan_D.execute_func()

if __name__ == '__main__':
    rospy.init_node('combine', anonymous=True)

    thread1 = Thread(target=thread_job)
    thread2 = Thread(target=execute_func)

    thread1.start()
    thread2.start()

    
