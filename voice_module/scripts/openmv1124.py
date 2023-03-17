#!/usr/bin/env python
# encoding: utf-8

import serial
from time import sleep
import rospy
from std_msgs.msg import Int32

class openmv_color():
	def __init__(self):
		self.a = "a".encode("utf-8")
		self.b = "b".encode("utf-8")
		self.c = "c".encode("utf-8")
		self.ans = [0, 0, 0]
		self.count = 0
		self.color = 0
        #self.arm_arrive_subscriber = rospy.Subscriber('/arm_arrive', Int32, self.arm_arrive_callback)
		self.color_publisher = rospy.Publisher('/color', Int32, queue_size = 10)
		self.ser = serial.Serial('/dev/ttyACM0', 9600)
		while 1:
			if(self.ser.inWaiting() != 0):
				print("inwaiting = ", self.ser.inWaiting())
				d = self.ser.readline()
				if d == 'a\n':
					print("d = a")
					self.ans[0] += 1
				elif d == 'b\n':
					print("d == b")
					self.ans[1] += 1
				elif d == 'c\n':
					print("d == c")
					self.ans[2] += 1
				else:
					print("error")
				print(d)
				print(d.decode("utf-8"))
				#print(int(d))
			else:
				print("none")
			#sleep(1)
			if(self.ans[0] > 3):
				self.color = 0
				self.color_publisher.publish(self.color)
				self.ans[0] = 0
				self.ans[1] = 0
				self.ans[2] = 0
				# self.count = 0
				#break
			elif(self.ans[1] > 3):
				self.color = 1
				self.color_publisher.publish(self.color)
				self.ans[0] = 0
				self.ans[1] = 0
				self.ans[2] = 0
				# self.count = 0
				# break
			elif(self.ans[2] > 3):
				self.color = 2
				self.color_publisher.publish(self.color)
				self.ans[0] = 0
				self.ans[1] = 0
				self.ans[2] = 0
				# self.count = 0
				# break
			else:
				pass

if __name__ == '__main__':
    rospy.init_node('openmv_color')
    openmv_color_test = openmv_color()
    rospy.spin()