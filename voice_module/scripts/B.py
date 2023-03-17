#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan,Imu
from std_msgs.msg import Int32,Float32
from tf.transformations import euler_from_quaternion
from math import *
from geometry_msgs.msg import Twist,Vector3
import time

class Test:
    def __init__(self):
         self.dist_left1 = 60
         self.dist_right1 = 60
         self.around_flag = 0
         self.start = 0
         self.now_angle = 0
         self.now_distance = 0
         self.y = 0.05
         self.trans_time = 0

        # 阈值
         self.thres = 55
         self.abs_thres = 120
        # 左危险距离
         self.danger_thres1 = 35
        # 右危险距离
         self.danger_thres2 = 35
        # 理想值
         self.best = 44
         self.danger_time = 0
         self.ahead_time = 0
         # 前进时间
         self.ahead = 3
         # 花盆计数
         self.num = 0

         self.stop_distance = 0

         self.stop_yaw = 0
         self.flag = 0
         self.x = 0.08

         # 左边在阈值内且右边在危险距离内
    def change_thres(self,num):
        if num == 2:
            self.thres = 60
            self.best = 45
            self.x = 0.14
            self.danger_thres1 = 40
            self.danger_thres2 = 32
            self.ahead = 4
        elif num == 3:
            self.thres = 60
            self.best = 45
            self.x = 0.05
            self.danger_thres1 = 40
            self.danger_thres2 = 32
            self.ahead = 0.5
        else:
            self.thres = 55
            self.best = 44
            self.x = 0.08
            self.danger_thres1 = 35
            self.danger_thres2 = 35
            self.ahead = 1

    def imu_callback(self,msg):
        global vel_pub,arrive_pub,dist_pub
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
        # 8平移前往前走一段距离
        if self.around_flag == 0:
            # 如果左边危险了，就右转
            if self.num == 0 or self.num == 1:
                if self.dist_left1 <= self.danger_thres1:
                    self.now_angle = yaw - 5
                    self.danger_time = time.time()
                    self.around_flag = 2
                # 如果右边危险了，就左转
                elif self.dist_right1 <= self.danger_thres2:
                    self.now_angle = yaw + 5
                    self.danger_time = time.time()
                    self.around_flag = 1
                # 左边大于危险距离且小于停车阈值，且右边大于危险距离，则该停车了
                elif (self.dist_left1 > self.danger_thres1 and self.dist_left1 < self.thres) and self.dist_right1 > self.danger_thres2:
                    self.now_distance = self.dist_left1
                    self.trans_time = time.time()
                    # 8前进一小段距离
                    self.around_flag = 8
                else:
                    vel_cmd.linear.x = self.x
            else:
                if self.dist_left1 + self.dist_right1 <= self.abs_thres:
                    self.now_distance = self.dist_left1
                    self.around_flag = 8
                else:
                    vel_cmd.linear.x = self.x
                    if abs(self.stop_yaw - yaw) < 180:
                        # rospy.loginfo("差小于180")
                        vel_cmd.angular.z = (self.stop_yaw - yaw)*0.01
                    else:
                        if self.stop_yaw < 0 and yaw > 0:
                            rospy.loginfo("异号,原先小于0,现在大于0")
                            vel_cmd.angular.z = (abs(self.stop_yaw)-abs(yaw))*0.01
                        elif self.stop_yaw > 0 and yaw < 0:
                            rospy.loginfo("异号,原先大于0,现在小于0")
                            rospy.loginfo("原来角度:[%f],现在角度:[%f]",self.stop_yaw,yaw)
                            vel_cmd.angular.z = -(abs(self.stop_yaw)-abs(yaw))*0.01
                            rospy.loginfo("角速度:[%f]",vel_cmd.angular.z)
        # 左转
        elif self.around_flag == 1:
            # 左转
            vel_cmd.angular.z = 0.3
            vel_cmd.linear.x = self.x
            if time.time()-self.danger_time > 3:
                # 进入角度恢复模式
                self.around_flag = 3
        # 右转
        elif self.around_flag == 2:
            # 右转
            vel_cmd.angular.z = -0.3
            vel_cmd.linear.x = self.x
            if time.time()-self.danger_time > 3:
                # 进入角度恢复模式
                self.around_flag = 3
        # 3开始恢复角度
        elif self.around_flag == 3:
            if abs(self.now_angle - yaw) < 180:
                # rospy.loginfo("差小于180")
                vel_cmd.angular.z = (self.now_angle - yaw)*0.01
            else:
                if self.now_angle < 0 and yaw > 0:
                    rospy.loginfo("异号,原先小于0,现在大于0")
                    vel_cmd.angular.z = (abs(self.now_angle)-abs(yaw))*0.01
                elif self.now_angle > 0 and yaw < 0:
                    rospy.loginfo("异号,原先大于0,现在小于0")
                    rospy.loginfo("原来角度:[%f],现在角度:[%f]",self.now_angle,yaw)
                    vel_cmd.angular.z = -(abs(self.now_angle)-abs(yaw))*0.01
                    rospy.loginfo("角速度:[%f]",vel_cmd.angular.z)
            if abs(abs(self.now_angle)-abs(yaw)) < 1:
                self.start = time.time()
                # 恢复角度完成，开始后退
                self.around_flag = 4
        # 4开启后退模式
        elif self.around_flag == 4:
            # 后退时间设为2秒
            if time.time() - self.start >= 2:
                if self.flag == 1:
                    self.flag = 0
                    self.around_flag = 6
                # 再次进入前进模式
                else:
                    self.around_flag = 0
            else:
                # 角度继续调整
                vel_cmd.angular.z = (self.now_angle-yaw)*0.01
                vel_cmd.linear.x = -self.x
        # 5正在浇水
        elif self.around_flag == 5:
            # 停
            arrive_pub.publish(1)
            dist_pub.publish(self.dist_left1)
            self.stop_distance = self.dist_left1
            if self.num == 1:
                self.stop_yaw = yaw
            self.around_flag = 10
            if self.around_flag == 10:
                vel_cmd.linear.x = 0
                vel_cmd.angular.z = 0
        # 6刚浇完水
        elif self.around_flag == 6:
            if self.num == 0 or self.num == 1:
            # 如果左边危险了，就右转
                if self.dist_left1 <= self.danger_thres1:
                    self.now_angle = yaw - 5
                    self.danger_time = time.time()
                    self.around_flag = 2
                    self.flag = 1
                # 如果右边危险了，就左转
                elif self.dist_right1 <= self.danger_thres2:
                    self.now_angle = yaw + 5
                    self.danger_time = time.time()
                    self.around_flag = 1
                    self.flag = 1
            # 如果左轮大于阈值，则置为还没浇水状态
            if self.dist_left1 - self.stop_distance > 10:
                rospy.loginfo("检测到缝隙")
                self.around_flag=0
            else:
            # 否则就一直走
                vel_cmd.linear.x = self.x
        # 7停车后开始调整位移量
        elif self.around_flag == 7:   
            # 目前的距离比理想距离小，则右移
            if self.now_distance - self.best < 0:
                rospy.loginfo("目前距离和理想距离的差:[%.1f]",self.now_distance-self.best)
                vel_cmd.linear.y = -self.y
            # 目前距离比理想距离大，则左移
            elif self.now_distance - self.best > 0:
                rospy.loginfo("目前距离和理想距离的差:[%.1f]",self.now_distance-self.best)
                vel_cmd.linear.y = self.y
            # 如果过了第1、2个花盆，平移时需要调整角度
            if self.num >= 2:
                if abs(self.stop_yaw - yaw) < 180:
                    # rospy.loginfo("差小于180")
                    vel_cmd.angular.z = (self.stop_yaw - yaw)*0.01
                else:
                    if self.stop_yaw < 0 and yaw > 0:
                        rospy.loginfo("异号,原先小于0,现在大于0")
                        vel_cmd.angular.z = (abs(self.stop_yaw)-abs(yaw))*0.01
                    elif self.stop_yaw > 0 and yaw < 0:
                        rospy.loginfo("异号,原先大于0,现在小于0")
                        rospy.loginfo("原来角度:[%f],现在角度:[%f]",self.stop_yaw,yaw)
                        vel_cmd.angular.z = -(abs(self.stop_yaw)-abs(yaw))*0.01
                        rospy.loginfo("角速度:[%f]",vel_cmd.angular.z)

            if time.time()-self.ahead_time >= abs(self.now_distance-self.best)*0.02/self.y:
                self.around_flag = 5
        # 前进一段距离
        elif self.around_flag == 8:
            if time.time()-self.trans_time >= self.ahead:
                self.ahead_time = time.time()
                self.around_flag = 7
            else:
                vel_cmd.linear.x = self.x
                # 如果过了第1、2个花盆，平移时需要调整角度
                if self.num >= 2:
                    if abs(self.stop_yaw - yaw) < 180:
                        # rospy.loginfo("差小于180")
                        vel_cmd.angular.z = (self.stop_yaw - yaw)*0.01
                    else:
                        if self.stop_yaw < 0 and yaw > 0:
                            rospy.loginfo("异号,原先小于0,现在大于0")
                            vel_cmd.angular.z = (abs(self.stop_yaw)-abs(yaw))*0.01
                        elif self.stop_yaw > 0 and yaw < 0:
                            rospy.loginfo("异号,原先大于0,现在小于0")
                            rospy.loginfo("原来角度:[%f],现在角度:[%f]",self.stop_yaw,yaw)
                            vel_cmd.angular.z = -(abs(self.stop_yaw)-abs(yaw))*0.01
                            rospy.loginfo("角速度:[%f]",vel_cmd.angular.z)
        
        # rospy.loginfo("状态:[%d],左距离:[%.1f],右距离:[%.1f]",self.around_flag,self.dist_left1,self.dist_right1)
        # rospy.loginfo("停止距离:[%f],第2花盆停止角度:[%f]",self.stop_distance,self.stop_yaw)
        # rospy.loginfo("左边理想值:[%f]"%self.best)
        # vel_pub.publish(vel_cmd)

    def LidarCallback(self,msg):
        if msg.ranges[350] < 6.00:
            self.dist_left1 = msg.ranges[350]*100

        if msg.ranges[925] < 6.00:
            self.dist_right1 = msg.ranges[925]*100
        # rospy.loginfo("左:%f, 右:%f",self.dist_left1, self.dist_right1)
    
    def waterCallback(self,msg):
        if msg.data == 1:
            # 变为浇完水模式
            self.around_flag = 6
            self.num += 1
            self.change_thres(self.num)
            rospy.loginfo("num:[%d]",self.num)

if __name__ == "__main__":
    rospy.init_node("lidar_node")
    test = Test()
    rospy.Subscriber("/scan",LaserScan,test.LidarCallback,queue_size=10)
    rospy.Subscriber("/imu_nine",Imu,test.imu_callback,queue_size=10)
    rospy.Subscriber("/judge",Int32,test.waterCallback,queue_size=10)
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    arrive_pub = rospy.Publisher("/arrive_info",Int32,queue_size=10)
    dist_pub = rospy.Publisher("/us_data",Float32,queue_size=10)
    rospy.spin()