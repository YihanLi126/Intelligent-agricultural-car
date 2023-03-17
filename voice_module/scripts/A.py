#!/usr/bin/env python
# encoding: utf-8

from numpy import around
import rospy
import time
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist,Vector3
# from voice_module.msg import DistanceMsg
from threading import Thread
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import *

class Voicer:
    def __init__(self):

        # 距离变量
        self.distance_left1 = 50
        self.distance_right1 = 50

        # 记录浇水完成和下一次停车间隔时间
        self.water_time = []
        # 左轮标志位
        self.left_flag = 0
        self.right_flag = 0
        # 浇水标志位
        self.back_leave = 1
        
        self.flag = 1
        # 计数
        self.num = 0
        # 记录循环次数
        self.i = 0
        self.status = 0
        self.water=0

        # 阈值
        self.thres = 45
        # 危险距离
        self.danger_thres = 30
        # 记录后退开始时间
        self.start = 0
        # 根据每次imu的朝向值确定理想朝向值
        # 0初始状态 1调整角度 2后退
        self.now_angle = 0
        # 记录调整位姿的状态
        self.around_flag = 0
        # imu信息
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # linear.x值
        self.x = 0.08

    def pub_isArrive(self,flag):
        arrive_pub=rospy.Publisher('/arrive_info',Int32,queue_size=10)
        dis_pub=rospy.Publisher('/us_data', Float32, queue_size=10)
        dis_pub.publish(self.distance_left1*100)
        arrive_pub.publish(flag)

    def pub_velocity(self,x, z):
               
        # 创建一个Publisher，发布名为/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
        turtle_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # 创建Twist()消息类型
        vel_msg = Twist()
        vel_msg.angular.z = z
        vel_msg.linear.x = x
        
        # while True:
        # 发布速度命令
        turtle_vel_pub.publish(vel_msg)
        
        if self.i%10 == 0:
            rospy.loginfo("Publsh velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)

    def LidarCallback(self, msg):
        if msg.ranges[350] != float('inf') and msg.ranges[350]<3.00:
            self.distance_left1 = msg.ranges[350]*100
        if msg.ranges[925] != float('inf') and msg.ranges[925]<3.00:
            self.distance_left1 = msg.ranges[925]*100 

    def imu_callback(self,msg):
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
        
    def thread_job(self):
        try:
            rospy.spin()
            print('thread1-rospy.spin')
        except rospy.ROSInterruptException:
            print('exception_subscribe')

    def waterCallBack(self, msg):
        if msg.data==1:
            self.water=1
            self.num+=1
            self.pub_isArrive(0)
            self.water_time.append(time.time())
            # rospy.loginfo(self.back_leave)
            rospy.loginfo('self.num:%d',self.num)
        else:
            rospy.loginfo('Failed')

    def water_Subscriber(self):
        # rospy.Subscriber("/water", Int32, self.waterCallBack, queue_size=10)
        rospy.Subscriber("/judge", Int32, self.waterCallBack, queue_size=10)
        

    # 判断距离状态
    def judge_status(self):
        # 左边小于危险距离
        if self.distance_left1 <= self.danger_thres:
            return 1
        # 右边小于危险距离
        elif self.distance_right1 <= self.danger_thres:
            return 2
        # 左右两边都在停车阈值之内
        elif self.distance_left1 >= self.danger_thres and self.distance_right1 >= self.danger_thres and self.distance_left1 <= self.thres and self.distance_right1 <= self.thres:
            return 3
        # 其余情况继续向前走
        else:
            return 4
    
    # 危险距离内调整状态
    def adjust(self,type):
        # 根据靠左靠右判断转向方式
        if type == 1:
           
            # 由于靠左，则调整位姿的角度应比现在的角度小
            self.now_angle = self.yaw - 7
            # 右转
            self.pub_velocity(0.08,-0.3)
            # 右转30度
            if self.now_angle - self.yaw > 30:
                # 进入角度恢复模式
                self.around_flag = 1
        if type == 2:
            # 由于靠右，则调整位姿的角度应比现在的角度大
            self.now_angle = self.yaw + 7
            # 左转
            self.pub_velocity(0.08,0.3)
            # 左转30度
            if self.yaw - self.now_angle > 30:
                    # 进入角度恢复模式
                    self.around_flag = 1
        
        # 角度恢复模式
        if self.around_flag == 1:
                self.pub_velocity(0.0,(self.now_angle - self.yaw)*0.01)
                if abs(self.now_angle-self.yaw) < 1:
                    self.start = time.time()
                    # 角度调整完成，进入后退模式
                    self.around_flag = 2
        # 开启后退模式
        elif self.around_flag == 2:
            # 后退时间设为5秒
            if time.time() - self.start >= 5:
                # 后退完成，进入初始状态
                self.around_flag = 0
            else:
                # 角度继续调整,同时后退
                self.pub_velocity(-0.08,(self.now_angle-self.yaw)*0.01)


    def move(self):
        # 获取机器人目前距离状态
        # 1左危险 2右危险 3可以停车 4前进
        pass
        '''self.front_flag = self.test_obs(self.distance_left1) 
        rospy.loginfo("water:[%d],front_flag:[%d],len(water_time):[%d]",self.water,self.front_flag,
        len(self.water_time))
        if self.water==0:
            if self.front_flag==1:
                if len(self.water_time)==0:
                    self.water_time.append(time.time())
                    self.pub_velocity(0.0,0.0)
                    # self.pub_isArrive(1)
                elif len(self.water_time) >= 2:
                    if time.time()-self.water_time[-1]<3:
                        self.pub_velocity(self.x,0.0)
                        self.water=1
                    else:
                        self.water_time.append(time.time())
                        self.pub_velocity(0.0,0.0)
                        rospy.loginfo("delta time:[%.0f]",self.water_time[-1]-self.water_time[-2])
                        self.water=2
                else:
                    self.water=2

            elif self.front_flag==0:
                # self.pub_isArrive(0)
                self.pub_velocity(self.x,0.0)
        elif self.water==1:
            if self.front_flag==0:
                self.water=0
            else:
                # self.pub_isArrive(0)
                self.pub_velocity(self.x,0.0)
        elif self.water==2:
            # self.pub_isArrive(1)
            self.pub_velocity(0.0,0.0)'''
    


    def execute_func(self):
        while not rospy.is_shutdown():
            self.i += 1
            self.move()
            time.sleep(0.02)
            if self.num == 6:
                break
            
       
               

if __name__ == '__main__':
    voicer = Voicer()
    rospy.init_node('A', anonymous=True)
    # 订阅雷达信息
    rospy.Subscriber("/scan",LaserScan,voicer.LidarCallback,queue_size=10)
    # 订阅imu信息
    rospy.Subscriber("/imu_nine",Vector3,voicer.imu_callback,queue_size=10)
    voicer.water_Subscriber()

    thread1 = Thread(target=voicer.thread_job)
    thread2 = Thread(target=voicer.execute_func)

    thread1.start()
    thread2.start()
