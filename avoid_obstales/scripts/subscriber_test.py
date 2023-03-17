#! /usr/bin/env python
#coding=utf-8

import rospy
import time
from std_msgs.msg import Int32
from threading import Thread
import psutil

num_sub1 = 0
num_sub2 = 0

class sub1:
	def __init__(self):
		self.sub = rospy.Subscriber('/test', Int32, self.sub_callback)
	def sub_callback(self, msg):
		global num_sub1
		num_sub1 += 1
		print("sub1 receive: ", num_sub1)

class sub2:
	def __init__(self):
		self.sub = rospy.Subscriber('/test', Int32, self.sub_callback)
	def sub_callback(self, msg):
		global num_sub2
		num_sub2 += 1
		print("sub2 receive: ", num_sub2)

def exec_():
	sub1_ = sub1()
	time.sleep(5)
	sub1_.unregister()
	#del(sub1.sub)
	#del(sub1_.sub_callback)
	print("delete sub1")
	sub2_ = sub2()
	time.sleep(5)
	sub2_.unregister()
	#del(sub2.sub)
	#del(sub2_.sub_callback)
	print("delete sub2")

def thread_():
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('subscriber_test')
	thread1 = Thread(target=exec_)
	thread2 = Thread(target=thread_)
	thread1.start()
	thread2.start()