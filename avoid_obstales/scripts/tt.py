#!/usr/bin/env python
# encoding: utf-8


# '''
# ########################################################### imu_nine start 
# import argparse

# # import numpy as np
# # import serial  # 导入模块
# import serial.tools.list_ports
# import threading
# import struct
# # import time
# # from copy import deepcopy
# # import sys
# # import os
# # import math

# from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE


# # 宏定义参数
# PI = 3.1415926
# FRAME_HEAD = str('fc')
# FRAME_END = str('fd')
# TYPE_IMU = str('40')
# TYPE_AHRS = str('41')
# TYPE_INSGPS = str('42')
# TYPE_GEODETIC_POS = str('5c')
# TYPE_GROUND = str('f0')
# IMU_LEN = str('38')  # //56
# AHRS_LEN = str('30')  # //48
# INSGPS_LEN = str('48')  # //80
# GEODETIC_POS_LEN = str('20')  # //32
# PI = 3.141592653589793
# DEG_TO_RAD = 0.017453292519943295


# # 获取命令行输入参数
# def parse_opt(known=False):

#     parser = argparse.ArgumentParser()
#     # parser.add_argument('--debugs', type=bool, default=False, help='if debug info output in terminal ')
#     parser.add_argument('--port', type=str, default='/ttyUSB1', help='the models serial port receive data')
#     parser.add_argument('--bps', type=int, default=921600, help='the models baud rate set on wincc')
#     parser.add_argument('--timeout', type=int, default=20, help='set the serial port timeout')
#     # parser.add_argument('--device_type', type=int, default=0, help='0: origin_data, 1: for single imu or ucar in ROS')

#     receive_params = parser.parse_known_args()[0] if known else parser.parse_args()
#     return receive_params


# # 接收数据线程
# def receive_data():
#     open_port()
#     # 尝试打开串口
#     try:
#         serial_ = serial.Serial(port=opt.port, baudrate=opt.bps, bytesize=EIGHTBITS, parity=PARITY_NONE,
#                                 stopbits=STOPBITS_ONE,
#                                 timeout=opt.timeout) 
#         print("baud rates = " + str(serial_.baudrate))
#     except:
#         print("error:  unable to open port .")
#         exit(1)
#     # 循环读取数据
#     while serial_.isOpen() and tr.is_alive():
#         # rbdata = ser.readline()
#         # # rbdata = ser.read_all()
#         #
#         # if len(rbdata) != 0:
#         #     rxdata = rbdata.hex()
#         #     print(rxdata)
#         check_head = serial_.read().hex()
#         # 校验帧头
#         if check_head != FRAME_HEAD:
#             continue
#         head_type = serial_.read().hex()
#         # 校验数据类型
#         if (head_type != TYPE_IMU and head_type != TYPE_AHRS and head_type != TYPE_INSGPS and
#                 head_type != TYPE_GEODETIC_POS and head_type != 0x50 and head_type != TYPE_GROUND):
#             continue
#         check_len = serial_.read().hex()
#         # 校验数据类型的长度
#         if head_type == TYPE_IMU and check_len != IMU_LEN:
#             continue
#         elif head_type == TYPE_AHRS and check_len != AHRS_LEN:
#             continue
#         elif head_type == TYPE_INSGPS and check_len != INSGPS_LEN:
#             continue
#         elif head_type == TYPE_GEODETIC_POS and check_len != GEODETIC_POS_LEN:
#             continue
#         elif head_type == TYPE_GROUND or head_type == 0x50:
#             continue
#         check_sn = serial_.read().hex()
#         head_crc8 = serial_.read().hex()
#         crc16_H_s = serial_.read().hex()
#         crc16_L_s = serial_.read().hex()

#         # 读取并解析IMU数据
#         if head_type == TYPE_IMU:
#             data_s = serial_.read(int(IMU_LEN, 16))
#             print("Gyroscope_X(rad/s): " + str(struct.unpack('f', data_s[0:4])[0]))
#             print("Gyroscope_Y(rad/s) : " + str(struct.unpack('f', data_s[4:8])[0]))
#             print("Gyroscope_Z(rad/s) : " + str(struct.unpack('f', data_s[8:12])[0]))
#             print("Accelerometer_X(m/s^2) : " + str(struct.unpack('f', data_s[12:16])[0]))
#             print("Accelerometer_Y(m/s^2) : " + str(struct.unpack('f', data_s[16:20])[0]))
#             print("Accelerometer_Z(m/s^2) : " + str(struct.unpack('f', data_s[20:24])[0]))
#             # print("Magnetometer_X(mG) : " + str(struct.unpack('f', data_s[24:28])[0]))
#             # print("Magnetometer_Y(mG) : " + str(struct.unpack('f', data_s[28:32])[0]))
#             # print("Magnetometer_Z(mG) : " + str(struct.unpack('f', data_s[32:36])[0]))
#             # print("IMU_Temperature : " + str(struct.unpack('f', data_s[36:40])[0]))
#             # print("Pressure : " + str(struct.unpack('f', data_s[40:44])[0]))
#             # print("Pressure_Temperature : " + str(struct.unpack('f', data_s[44:48])[0]))
#             # print("Timestamp(us) : " + str(struct.unpack('ii', data_s[48:56])[0]))
#         # 读取并解析AHRS数据
#         elif head_type == TYPE_AHRS:
#             data_s = serial_.read(int(AHRS_LEN, 16))
#             print("RollSpeed(rad/s): " + str(struct.unpack('f', data_s[0:4])[0]))
#             print("PitchSpeed(rad/s) : " + str(struct.unpack('f', data_s[4:8])[0]))
#             print("HeadingSpeed(rad) : " + str(struct.unpack('f', data_s[8:12])[0]))
#             print("Roll(rad) : " + str(struct.unpack('f', data_s[12:16])[0]))
#             print("Pitch(rad) : " + str(struct.unpack('f', data_s[16:20])[0]))
#             print("Heading(rad) : " + str(struct.unpack('f', data_s[20:24])[0]))
#             print("Q1 : " + str(struct.unpack('f', data_s[24:28])[0]))
#             print("Q2 : " + str(struct.unpack('f', data_s[28:32])[0]))
#             print("Q3 : " + str(struct.unpack('f', data_s[32:36])[0]))
#             print("Q4 : " + str(struct.unpack('f', data_s[36:40])[0]))
#             # print("Timestamp(us) : " + str(struct.unpack('ii', data_s[40:48])[0]))
#         # 读取并解析INSGPS数据
#         elif head_type == TYPE_INSGPS:
#             data_s = serial_.read(int(INSGPS_LEN, 16))
#             print("BodyVelocity_X:(m/s)" + str(struct.unpack('f', data_s[0:4])[0]))
#             print("BodyVelocity_Y:(m/s)" + str(struct.unpack('f', data_s[4:8])[0]))
#             print("BodyVelocity_Z:(m/s)" + str(struct.unpack('f', data_s[8:12])[0]))
#             print("BodyAcceleration_X:(m/s^2)" + str(struct.unpack('f', data_s[12:16])[0]))
#             print("BodyAcceleration_Y:(m/s^2)" + str(struct.unpack('f', data_s[16:20])[0]))
#             print("BodyAcceleration_Z:(m/s^2)" + str(struct.unpack('f', data_s[20:24])[0]))
#             print("Location_North:(m)" + str(struct.unpack('f', data_s[24:28])[0]))
#             print("Location_East:(m)" + str(struct.unpack('f', data_s[28:32])[0]))
#             print("Location_Down:(m)" + str(struct.unpack('f', data_s[32:36])[0]))
#             print("Velocity_North:(m)" + str(struct.unpack('f', data_s[36:40])[0]))
#             print("Velocity_East:(m/s)" + str(struct.unpack('f', data_s[40:44])[0]))
#             print("Velocity_Down:(m/s)" + str(struct.unpack('f', data_s[44:48])[0]))
#             # print("Acceleration_North:(m/s^2)" + str(struct.unpack('f', data_s[48:52])[0]))
#             # print("Acceleration_East:(m/s^2)" + str(struct.unpack('f', data_s[52:56])[0]))
#             # print("Acceleration_Down:(m/s^2)" + str(struct.unpack('f', data_s[56:60])[0]))
#             # print("Pressure_Altitude:(m)" + str(struct.unpack('f', data_s[60:64])[0]))
#             # print("Timestamp:(us)" + str(struct.unpack('ii', data_s[64:72])[0]))
#         # 读取并解析GPS数据
#         elif head_type == TYPE_GEODETIC_POS:
#             data_s = serial_.read(int(GEODETIC_POS_LEN, 16))
#             print(" Latitude:(rad)" + str(struct.unpack('d', data_s[0:8])[0]))
#             print("Longitude:(rad)" + str(struct.unpack('d', data_s[8:16])[0]))
#             print("Height:(m)" + str(struct.unpack('d', data_s[16:24])[0]))


# # 寻找输入的port串口
# def find_serial():
#     port_list = list(serial.tools.list_ports.comports())
#     for port in port_list:
#         if port.name.lower() == opt.port.lower():
#             return True
#     return False


# def open_port():
#     if find_serial():
#         print("find this port : " + opt.port)
#     else:
#         print("error:  unable to find this port : " + opt.port)
#         exit(1)


# if __name__ == "__main__":
#     opt = parse_opt()
#     tr = threading.Thread(target=receive_data)
#     tr.start()
    
# ########################################################### imu_nine end
# '''

# #####################################################laserscan test begin
# import rospy
# import time
# from std_msgs.msg import Int32
# from geometry_msgs.msg import Twist
# from threading import Thread
# from sensor_msgs.msg import LaserScan
# def callback(msg):
#     print(msg.ranges[0])
# rospy.init_node('laserscan_analyze')
# laser_subscriber = rospy.Subscriber('/scan', LaserScan, callback)
# rospy.spin()

# #####################################################laserscan test end
# ############################################################## openmv begin
# # import serial
# # from time import sleep
# # import rospy
# # from std_msgs.msg import Int32

# # class openmv_color():
# #     def __init__(self):
# #         self.a = "a".encode("utf-8")
# #         self.b = "b".encode("utf-8")
# #         self.c = "c".encode("utf-8")
# #         self.ans = [0, 0, 0]
# #         self.count = 0
# #         self.color = 0
# #         self.arm_arrive_subscriber = rospy.Subscriber('/arm_arrive', Int32, self.arm_arrive_callback)
# #         self.color_publisher = rospy.Publisher('/color', Int32, queue_size = 10)
# #         self.ser = serial.Serial('/dev/ttyACM0', 9600)
# #         self.arrive_info_subscriber = rospy.Subscriber('/arrive_info', Int32, self.arrive_info_callback)

# #     def arm_arrive_callback(self, msg):
# #         if(self.count == 0):
# #             self.count = 1
# #             while 1:
# #                 if(self.ser.inWaiting() != 0):
# #                     print("inwaiting = ", self.ser.inWaiting())
# #                     d = self.ser.readline()
# #                     if d == 'a\n':
# #                         print("d = a")
# #                         self.ans[0] += 1
# #                     elif d == 'b\n':
# #                         print("d == b")
# #                         self.ans[1] += 1
# #                     elif d == 'c\n':
# #                         print("d == c")
# #                         self.ans[2] += 1
# #                     else:
# #                         print("error")
# #                     print(d)
# #                     print(d.decode("utf-8"))
# #                     #print(int(d))
# #                 else:
# #                     print("none")
# #                 #sleep(1)
# #                 if(self.ans[0] > 3):
# #                     self.color = 0
# #                     self.color_publisher.publish(self.color)
# #                     self.ans[0] = 0
# #                     self.ans[1] = 0
# #                     self.ans[2] = 0
# #                     self.count = 0
# #                     break
# #                 elif(self.ans[1] > 3):
# #                     self.color = 1
# #                     self.color_publisher.publish(self.color)
# #                     self.ans[0] = 0
# #                     self.ans[1] = 0
# #                     self.ans[2] = 0
# #                     self.count = 0
# #                     break
# #                 elif(self.ans[2] > 3):
# #                     self.color = 2
# #                     self.color_publisher.publish(self.color)
# #                     self.ans[0] = 0
# #                     self.ans[1] = 0
# #                     self.ans[2] = 0
# #                     self.count = 0
# #                     break
# #                 else:
# #                     pass
# #         else:
# #             print("receive arm_arrive")

# # if __name__ == '__main__':
# #     rospy.init_node('openmv_color')
# #     openmv_color_test = openmv_color()
# #     rospy.spin()
        
# #################################################################### openmv end
# '''
# if __name__ == '__main__':
# 	ser = serial.Serial('/dev/ttyUSB0', 9600, timeout = 0.5)
# 	begin = 0x03
# 	stop = bytes([100, 2, 4, 24])
# 	while True:
# 		size = ser.inWaiting()
# 		if(size != 0):
# 			res = ser.read(size)
# 			print(res)
# 			ser.flushInput()
# 		ser.write(stop)
# 		ser.write(begin)
# 		print("write already")	
# 		sleep(1)

# import serial
# from time import sleep
# ls = ['c', 's', 'q', 'h', 'z', 'y', '0', '1', '2']
# i = 0
# ser = serial.Serial('/dev/ttyUSB0', 9600, timeout = 0.5)	
# def write(seri, lsp, ip):
#   seri.write(lsp[ip].encode("utf8"))
#   print('have writen', lsp[ip])
# def read(seri):
#   ans = seri.read()
#   print('have read', ans)
# if __name__ == '__main__':
#   while(True):
#     #write(ser, ls, i)
#     read(ser)
#     i = (i + 1)%9
#     sleep(1)

# ###################################################################### voice module test begin

import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import math
GPIO.setmode(GPIO.BCM)

light_switch = 17
GPIO.setup(light_switch, GPIO.IN)
rospy.init_node('light_switch')
rate = rospy.Rate(5)

# while(1):
# 	print(time.time(), ':', GPIO.input(light_switch))
# 	rate.sleep()

TRIG_left1 = 12
ECHO_left1 = 16
TRIG_left2 = 4
ECHO_left2 = 17
TRIG_right1 = 27
ECHO_right1 = 22
TRIG_right2 = 9
ECHO_right2 = 10
GPIO.setup(TRIG_left1, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(ECHO_left1, GPIO.IN)
GPIO.setup(TRIG_left2, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(ECHO_left2, GPIO.IN)
GPIO.setup(TRIG_right1, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(ECHO_right1, GPIO.IN)
GPIO.setup(TRIG_right2, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(ECHO_right2, GPIO.IN)

def measure():
	GPIO.output(TRIG_left1, True)
	time.sleep(0.00001)
	GPIO.output(TRIG_left1, False)
	while GPIO.input(ECHO_left1)==0:
		pass
	start = time.time()

	while GPIO.input(ECHO_left1)==1:
		pass
	end = time.time()
	distance = round((end-start)*343/2*100, 2)
	print('left1 distance:', distance)
 
	GPIO.output(TRIG_left2, True)
	time.sleep(0.00001)
	GPIO.output(TRIG_left2, False)
	while GPIO.input(ECHO_left2)==0:
		pass
	start = time.time()
	while GPIO.input(ECHO_left2)==1:
		pass
	end = time.time()
	distance=round((end-start)*343/2*100, 2)
	print('left2 distance:', distance)
 
	GPIO.output(TRIG_right1, True)
	time.sleep(0.00001)
	GPIO.output(TRIG_right1, False)
	while GPIO.input(ECHO_right1)==0:
		pass
	start = time.time()
	while GPIO.input(ECHO_right1)==1:
		pass
	end = time.time()
	distance=round((end-start)*343/2*100, 2)
	print('right1 distance:', distance)
 
	GPIO.output(TRIG_right2, True)
	time.sleep(0.00001)
	GPIO.output(TRIG_right2, False)
	while GPIO.input(ECHO_right2)==0:
		pass
	start = time.time()
	while GPIO.input(ECHO_right2)==1:
		pass
	end = time.time()
	distance=round((end-start)*343/2*100, 2)
	print('right2 distance:', distance)

while True:
	measure()
	time.sleep(1)
GPIO.cleanup()

# ###################################################################### voice module test end
# '''