#!/usr/bin/env python
# -*- coding: utf-8 -*-
from gettext import translation
import rospy
from sensor_msgs.msg import LaserScan,Imu
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion
from math import *
from geometry_msgs.msg import Twist,Vector3
import time

class Test:
    def __init__(self):
         self.dist_left1 = 55
         self.dist_right1 = 55
         self.around_flag = 0
         self.start = 0
         self.now_angle = 0
         self.now_distance = 0
         self.angular = 0.05
         self.trans_time = 0

        #  self.now_dist = 0.5
        #  self.best = 0.40
        #  self.error = 0
        #  self.new_error = 0
        #  self.water = 0

        # 阈值
         self.thres = 45
        # 左危险距离
         self.danger_thres1 = 32
        # 右危险距离
         self.danger_thres2 = 28
        # 理想值
         self.best = 40

         # 花盆计数
         self.num = 0

    def imu_callback(self,msg):
        global vel_pub
        if msg.orientation_covariance[0]<0:
            return
        # 欧拉角-弧度转角度
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        roll = roll*180/pi
        pitch = pitch*180/pi
        yaw = yaw*180/pi
        # rospy.loginfo("yaw:[%.1f]",yaw)
        vel_cmd = Twist()
        # 0还没浇水 1左转  2右转  3调整角度  4后退
        # 5正在浇水  6刚浇完水  7浇水前调整平移
        
        # 0还没浇水
        if self.around_flag == 0:
            vel_cmd.linear.x = 0.08
            # 左边大于危险距离且小于停车阈值，且右边大于危险距离，则该停车了
            if self.dist_left1 > self.danger_thres1 and self.dist_left1 < self.thres and self.dist_right1 > self.danger_thres2:
                self.now_distance = self.dist_left1
                self.trans_time = time.time()
                # 7调整平移量
                self.around_flag = 7
            # 如果左边危险了，就右转
            elif self.dist_left1 <= self.danger_thres1:
                self.now_angle = yaw - 7
                self.around_flag = 2
            # 如果右边危险了，就左转
            elif self.dist_right1 <= self.danger_thres2:
                self.now_angle = yaw + 7
                self.around_flag = 1
        # 左转
        elif self.around_flag == 1:
            # 左转
            vel_cmd.angular.z = 0.3
            vel_cmd.linear.x = 0.08
            if yaw - self.now_angle > 30:
                # 进入角度恢复模式
                self.around_flag = 3
        # 右转
        elif self.around_flag == 2:
            # 右转
            vel_cmd.angular.z = -0.3
            vel_cmd.linear.x = 0.08
            # 右转30度
            if self.now_angle - yaw > 30:
                # 进入角度恢复模式
                self.around_flag = 3
        # 3开始恢复角度
        elif self.around_flag == 3:
            vel_cmd.angular.z = (self.now_angle - yaw)*0.01
            if abs(self.now_angle-yaw) < 1:
                self.start = time.time()
                # 恢复角度完成，开始后退
                self.around_flag = 4
        # 4开启后退模式
        elif self.around_flag == 4:
            # 后退时间设为6秒
            if time.time() - self.start >= 6:
                # 再次进入前进模式
                self.around_flag = 0
            else:
                # 角度继续调整
                vel_cmd.angular.z = (self.now_angle-yaw)*0.01
                # 同时后退
                vel_cmd.linear.x = -0.08
        # 5正在浇水
        elif self.around_flag == 5:
            # 停
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0
        # 6刚浇完水
        elif self.around_flag == 6:
            # 如果左轮大于阈值，则置为还没浇水状态
            if self.dist_left1 > self.thres:
                self.around_flag=0
            else:
            # 否则就一直走呗
                vel_cmd.linear.x = 0.08
        # 7停车后开始调整位移量
        elif self.around_flag == 7:
            # 目前的距离比理想距离小，则右移
            if self.now_distance - self.best < 0:
                vel_cmd.linear.y = -self.angular
            # 目前距离比理想距离大，则左移
            elif self.now_distance - self.best > 0:
                vel_cmd.linear.y = self.angular
            if time.time()-self.trans_time >= abs(self.now_distance-self.best)*0.015/self.angular:
                print(time.time()-self.trans_time)
                print(abs(self.now_distance-self.best)*0.05/self.angular)
                self.around_flag = 5

        rospy.loginfo("around_flag:[%d]",self.around_flag)
        vel_pub.publish(vel_cmd)

    def LidarCallback(self,msg):
        if msg.ranges[350] < 3.00:
            self.dist_left1 = msg.ranges[350]*100


        if msg.ranges[925] < 3.00:
            self.dist_right1 = msg.ranges[925]*100
        # rospy.loginfo("左:%f, 右:%f",self.dist_left1, self.dist_right1)
    
    def waterCallback(self,msg):
        if msg.data == 1:
            # 变为浇完水模式
            self.around_flag = 6
            self.num += 1
            rospy.loginfo("num:[%d]",self.num)

if __name__ == "__main__":
    rospy.init_node("lidar_node")
    test = Test()
    rospy.Subscriber("/scan",LaserScan,test.LidarCallback,queue_size=10)
    rospy.Subscriber("/imu_nine",Imu,test.imu_callback,queue_size=10)
    rospy.Subscriber("/judge",Int32,test.waterCallback,queue_size=10)
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    rospy.spin()