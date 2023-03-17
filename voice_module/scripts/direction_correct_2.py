#! /usr/bin/env python
# coding=utf-8

import rospy
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Point, Quaternion
from threading import Thread
from std_srvs.srv import Empty
import Queue
import tf
from sensor_msgs.msg import Imu

# import voice_test

cmd_vel_msg = Twist()


class Direction_Corrector():
    def __init__(self):
        self.Kp_dir = rospy.get_param('/Kp_dir_param')
        self.Ki_dir = rospy.get_param('/Ki_dir_param')
        self.Kd_dir = rospy.get_param('/Kd_dir_param')
        self.delta_ang = 0.0
        self.delta_ang_last = 0.0
        self.SumErrAng = 0.0
        self.flag = 0

        self.direction_init = 0.0
        self.odom = Quaternion()  # odom msg of the exact time
        self.angular_now = Queue.Queue(20)  # a queue storing the latest angular data
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.trash = 0.0

        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.x = 0
        cmd_vel_msg.angular.y = 0
        cmd_vel_msg.linear.x = 0.2
        cmd_vel_msg.linear.y = 0

    def thread_job(self):
        try:
            rospy.spin()
            print('thread1-rospy.spin')
        except rospy.ROSInterruptException:
            print('exception_subscribe')

    def imu_callback(self, msg):
        (r, p, y) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        if (self.angular_now.full()):
            # i = 0
            # while(i < 3):
            #    self.trash = self.trash + self.angular_now.get()
            #    i = i + 1
            # self.trash = self.trash / 3
            self.trash = self.angular_now.get()
        self.angular_now.put(y)
        print("from imu_callback:self.trash = ", self.trash)
        if(self.flag == 1):
            self.delta_ang = self.direction_init - self.trash
            self.SumErrAng = self.SumErrAng + self.delta_ang
            self.delta_ang_last = self.delta_ang
            print("thread2-excecute---delta_Ang:", self.delta_ang)

            if (abs(self.delta_ang) >= 0.1):
                print('KP_dir:', self.Kp_dir, '  ', 'delta_ang:', self.delta_ang)
                print('multiple:', self.Kp_dir * self.delta_ang)
                cmd_vel_msg.angular.z = self.Kp_dir * self.delta_ang + self.Ki_dir * self.SumErrAng + self.Kd_dir * (
                        self.delta_ang - self.delta_ang_last)
                if (abs(cmd_vel_msg.angular.z) >= 0.2):
                    if (cmd_vel_msg.angular.z > 0):
                        cmd_vel_msg.angular.z = 0.2
                    else:
                        cmd_vel_msg.angular.z = -0.2
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            print("cmd_vel_publish:", cmd_vel_msg.angular.z)

    def get_direction_start(self):
        while (self.trash == 0.0):
            time.sleep(0.1)
        rate = rospy.Rate(10)
        i = 0
        while (i < 30):
            self.direction_init = self.direction_init + self.angular_now.get()
            i = i + 1
            rate.sleep()
        self.direction_init = self.direction_init / 50
        print("get_direction_start:", self.direction_init)
        self.flag = 1

    def stable_angular_z(self):
        rate = rospy.Rate(20)

        try:
            while not rospy.is_shutdown():
                self.delta_ang = self.direction_init - self.trash
                self.SumErrAng = self.SumErrAng + self.delta_ang
                self.delta_ang_last = self.delta_ang
                print("thread2-excecute---delta_Ang:", self.delta_ang)

                if (abs(self.delta_ang) >= 0.1):
                    print('KP_dir:', self.Kp_dir, '  ', 'delta_ang:', self.delta_ang)
                    print('multiple:', self.Kp_dir * self.delta_ang)
                    cmd_vel_msg.angular.z = self.Kp_dir * self.delta_ang + self.Ki_dir * self.SumErrAng + self.Kd_dir * (
                                self.delta_ang - self.delta_ang_last)
                    if (abs(cmd_vel_msg.angular.z) >= 0.2):
                        if (cmd_vel_msg.angular.z > 0):
                            cmd_vel_msg.angular.z = 0.2
                        else:
                            cmd_vel_msg.angular.z = -0.2
                self.cmd_vel_publisher.publish(cmd_vel_msg)
                print("cmd_vel_publish:", cmd_vel_msg.angular.z)
                # voicer.get_distance()
                # voicer.judgeIsGo()
                rate.sleep()

        except rospy.ROSInterruptException:
            print('exception_publish')

    def execute_func(self):
        self.get_direction_start()
        #self.stable_angular_z()


if __name__ == '__main__':
    rospy.init_node('direction_correct')
    direction_corrector = Direction_Corrector()
    direction_corrector.execute_func()
    # voicer = voice_test.Voicer()

    thread1 = Thread(target=direction_corrector.thread_job)
    thread2 = Thread(target=direction_corrector.execute_func)

    thread1.start()
    thread2.start()
