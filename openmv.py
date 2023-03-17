#!/usr/bin/env python
# encoding: utf-8

import rospy
import serial
import RPi.GPIO as GPIO
from std_msgs.msg import Int32
import time


ser = serial.Serial('/dev/ttyAMA0', 9600)
if ser.isOpen == False:
    ser.open()



class Openmv_Reciever:
    def __init__(self):
        self.pin1 = 38
        self.pin2 = 40


        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin1, GPIO.IN)
        GPIO.setup(self.pin2, GPIO.IN)
        

        self.info_pub = rospy.Publisher('/info_img', Int32, queue_size=10)
        self.info_msg = self.get_input()


    def get_input(self):
        cnt = 0
        while True:
            print cnt
            cnt = cnt+1
            print ser.name
            print ser.port
            print ser.baudrate
            print ser.bytesi ze
            print "----"
            '''
            print GPIO.input(self.pin1)
            print GPIO.input(self.pin2)
            '''
            size = ser.inWaiting()
            if size != 0:
                response = ser.read(size)
                print response
                self.flushInput()
                time.sleep(0.1)
            

    def publish(self):
        self.info_pub.publish(self.info_msg)



if __name__ == "__main__":
    rospy.init_node('Openmv_Reciever')
    orer = Openmv_Reciever()
    orer.get_input()


