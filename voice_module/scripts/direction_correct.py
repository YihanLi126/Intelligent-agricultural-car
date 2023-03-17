#! /usr/bin/env python
#coding=utf-8
'''
import rospy
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from threading import Thread
from sensor_msgs.msg import LaserScan
from simple_pid import PID


class Adjuster:
    def __init__(self):
        self.min_angular = 0
        self.ang_error = 0
        self.ang_refer = 80 #正前方的角度，需要再确认
        self.ang_v = 0 #输出角速度

    def LidarCallback(self, msg):
    	#开始扫描角度，需要再确认
    	start = 74
    	end = 86
    	self.angular = start
    	for i in range(start, end+1):
    		if msg.ranges[i] < msg.ranges[min_angular]:
    			self.min_angular = i
    			self.ang_error = self.min_angular - self.ang_refer

    def read_radar(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            print('exception_read_radar')

    def pub_angular(self):
    	#利用PID得到输出ang_v，输入是角度误差ang_error，目标为0
    	pid = PID(2, 0.01, 0.1, setpoint=0)
    	ang_pub = rospy.Publisher('/angular', Int32, queue_size=10)
        while not rospy.is_shutdown():
        	self.ang_v = pid(self.ang_error)
        	ang_pub.publish(self.ang_v)
        	



if __name__ == '__main__':
    adjuster = Adjuster()
    rospy.init_node('adjust', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, adjuster.LidarCallback, queue_size=10)

    thread1 = Thread(target=adjuster.read_radar)
    thread2 = Thread(target=adjuster.pub_angular)
    thread1.start()
    thread2.start()
'''
from curses import KEY_DC
import rospy
import time
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3
from threading import Thread
from std_srvs.srv import Empty
import Queue
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan

cmd_vel_msg = Twist()

cmd_vel_msg.linear.z = 0
cmd_vel_msg.angular.x = 0
cmd_vel_msg.angular.y = 0
cmd_vel_msg.linear.x = 0
cmd_vel_msg.linear.y = 0

direction_flag = 0

class Distance_Corrector():
    def __init__(self):
        self.Kp_dir = rospy.get_param('/Kp_dir_param')
        self.Ki_dir = rospy.get_param('/Ki_dir_param')
        self.Kd_dir = rospy.get_param('/Kd_dir_param')
        self.laserscan_subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)
        self.length_init = 0.0
        self.length_move = 1.0
        self.length_goal = self.length_init - self.length_move
        self.laser_count = 0
        self.delta_length = 0.0
        self.delta_length_last = 0.0
        self.SumErrLen = 0.0
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    def laserscan_callback(self, msg):
        global cmd_vel_msg
        if(self.laser_count < 200):
            self.length_init += msg.ranges[0]
            self.laser_count += 1
            self.cmd_vel_publisher.publish(cmd_vel_msg)
        else:
            if(self.laser_count == 200):
                self.length_init = self.length_init / 200
                self.laser_count += 1
                self.length_goal = self.length_init - self.length_move
            else:
                self.delta_len = msg.ranges[0] - self.length_goal
                self.SumErrLen = self.SumErrLen + self.delta_len
                self.delta_length_last = self.delta_len

                if(abs(self.delta_len) >= 0.01):
                    cmd_vel_msg.linear.x = self.Kp_dir * self.delta_len + self.Ki_dir * self.SumErrLen + self.Kd_dir * (self.delta_len - self.delta_length_last)
                    if(abs(cmd_vel_msg.linear.x) >= 0.08):
                        if(cmd_vel_msg.linear.x > 0):
                            cmd_vel_msg.linear.x = 0.08
                        else:
                            cmd_vel_msg.linear.x = -0.08
                    self.cmd_vel_publisher.publish(cmd_vel_msg)

class Direction_Corrector():
    def __init__(self):
        print("init class Direction_Corrector ing")
        #self.voicer = voice_test.Voicer()
        #self.voicer.water_Subscriber()
        self.Kp_dir = rospy.get_param('/Kp_dir_param')
        self.Ki_dir = rospy.get_param('/Ki_dir_param')
        self.Kd_dir = rospy.get_param('/Kd_dir_param')
        self.z_angular = Float64()
        self.delta_ang = 0.0
        self.delta_ang_last = 0.0
        self.SumErrAng = 0.0

        self.direction_init = 0.0
        self.odom = Quaternion() # odom msg of the exact time
        self.angular_now = Queue.Queue(20) # a queue storing the latest angular data
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        #self.imu_subscriber = rospy.Subscriber('/imu_nine', Imu, self.imu_callback)
        self.z_angular_publisher = rospy.Publisher('/z_angular', Float64, queue_size = 10)
        self.trash = 0.0
        
        self.count_euler = 0
        self.init_angel = 0.0
        self.euler_angle_subscriber = rospy.Subscriber('/euler_angles', Vector3, self.euler_angle_callback)

    def euler_angle_callback(self, msg):
        cmd_vel_msg.angular.z = 0.0
        print("reveive euler_angle_data")
        if(self.count_euler < 500):
            self.init_angel += msg.z
            self.count_euler += 1
        else:
            cmd_vel_msg.linear.x = 0
            if(self.count_euler == 500):
                self.init_angel = self.init_angel / 500
                self.count_euler += 1
                self.init_angel += (3.1415927 / 2)
                global direction_flag
                direction_flag = 1
            else:
                i = 0
                self.delta_ang = - self.init_angel + msg.z
                self.SumErrAng = self.SumErrAng + self.delta_ang
                self.delta_ang_last = self.delta_ang
                print("thread2-excecute---delta_Ang:", self.delta_ang)
                
                if(abs(self.delta_ang) >= 0.03):
                    print('KP_dir:', self.Kp_dir, '  ', 'delta_ang:', self.delta_ang)
                    print('multiple:', self.Kp_dir * self.delta_ang)
                    cmd_vel_msg.angular.z = self.Kp_dir * self.delta_ang + self.Ki_dir * self.SumErrAng + self.Kd_dir * (self.delta_ang - self.delta_ang_last)
                    if(abs(cmd_vel_msg.angular.z) >= 0.2):
                        if(cmd_vel_msg.angular.z > 0):
                            cmd_vel_msg.angular.z = 0.2
                        else:
                            cmd_vel_msg.angular.z = -0.2
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            # 以固定频率打印相关信息
            i += 1
            if i == 20:
                i = 0
            print("cmd_vel_publish:", cmd_vel_msg.angular.z)
            #rospy.loginfo("distance_left1:", self.voicer.distance_left1)
            #rospy.loginfo("distance_left2:", self.voicer.distance_left2)
            #rospy.loginfo("distance_right1:", self.voicer.distance_right1)
            #rospy.loginfo("distance_right2:", self.voicer.distance_right2)
               
            #self.voicer.get_distance()
            #self.voicer.judgeIsGo(cmd_vel_msg.angular.z)


    
    def thread_job(self):
        try:
            rospy.spin()
            print('thread1-rospy.spin')
        except rospy.ROSInterruptException:
            print('exception_subscribe')

    def imu_callback(self, msg):
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.z_angular.data = y
        self.z_angular_publisher.publish(self.z_angular)
        if(self.angular_now.full()):
            #i = 0
            #while(i < 3):
            #    self.trash = self.trash + self.angular_now.get()
            #    i = i + 1
            #self.trash = self.trash / 3
            self.trash = self.angular_now.get()
        self.angular_now.put(y)       
        print("from imu_callback:self.trash = ", self.trash)
        
    def get_direction_start(self):
        while(self.trash == 0.0):
            time.sleep(0.1)
        rate = rospy.Rate(10)
        i = 0
        while( i < 30):
            self.direction_init = self.direction_init + self.angular_now.get()
            i = i + 1
            rate.sleep()
        self.direction_init = self.direction_init / 50
        print("get_direction_start:", self.direction_init)

    def stable_angular_z(self):
        try:
            i = 0
            while not rospy.is_shutdown():
            # while i < 21:
                self.delta_ang = self.direction_init - self.trash
                self.SumErrAng =self.SumErrAng + self.delta_ang
                self.delta_ang_last = self.delta_ang
                print("thread2-excecute---delta_Ang:", self.delta_ang)
                
                if(abs(self.delta_ang) >= 0.05):
                    print('KP_dir:', self.Kp_dir, '  ', 'delta_ang:', self.delta_ang)
                    print('multiple:', self.Kp_dir * self.delta_ang)
                    cmd_vel_msg.angular.z = self.Kp_dir * self.delta_ang + self.Ki_dir * self.SumErrAng + self.Kd_dir * (self.delta_ang - self.delta_ang_last)
                    if(abs(cmd_vel_msg.angular.z) >= 0.2):
                        if(cmd_vel_msg.angular.z > 0):
                            cmd_vel_msg.angular.z = 0.2
                        else:
                            cmd_vel_msg.angular.z = -0.2
                self.cmd_vel_publisher.publish(cmd_vel_msg)
                # 以固定频率打印相关信息
                i += 1
                if i == 20:
                    i = 0
                rospy.loginfo("cmd_vel_publish:", cmd_vel_msg.angular.z)
                #rospy.loginfo("distance_left1:", self.voicer.distance_left1)
                #rospy.loginfo("distance_left2:", self.voicer.distance_left2)
                #rospy.loginfo("distance_right1:", self.voicer.distance_right1)
                #rospy.loginfo("distance_right2:", self.voicer.distance_right2)
                    
                #self.voicer.get_distance()
                #self.voicer.judgeIsGo(cmd_vel_msg.angular.z)
                time.sleep(0.02)
               
    
        except rospy.ROSInterruptException:
            print('exception_publish')
        except:
            print('other errors')
        finally:
            print('thread2 died')
        
    def execute_func(self):
        self.get_direction_start()
        self.stable_angular_z()
            
'''
if __name__ == '__main__':
    rospy.init_node('direction_correct')
    direction_corrector = Direction_Corrector()
    #direction_corrector.execute_func()

    thread1 = Thread(target=direction_corrector.thread_job)
    thread2 = Thread(target=direction_corrector.execute_func)

    thread1.start()
    thread2.start()
'''

if __name__ == '__main__':
    rospy.init_node('direction_correct')
    print("init node direction_correct successfully")
    direction_corrector = Direction_Corrector()
    while(direction_flag == 0):
        pass
    distance_corrector = Distance_Corrector()
    rospy.spin()