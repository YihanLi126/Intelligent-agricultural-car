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
		self.flag = 0
	def sub_callback(self, msg):
		if(self.flag == 0):
			global num_sub1
			num_sub1 += 1
			print("sub1 receive: ", num_sub1)
	def delete(self):
		self.flag = 1
class sub2:
	def __init__(self):
		self.sub = rospy.Subscriber('/test', Int32, self.sub_callback)
	def sub_callback(self, msg):
		global num_sub2
		num_sub2 += 1
		print("sub2 receive: ", num_sub2)
	def delete(self):
		self.sub = rospy.Subscriber('/win', Int32, self.win_callback)
		print("sub1 delete")
	def win_callback(self, msg):
		pass

def exec_():
	sub1_ = sub1()
	time.sleep(5)
	sub1_.delete()
	sub2_ = sub2()
	time.sleep(5)
	sub2_.delete()
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