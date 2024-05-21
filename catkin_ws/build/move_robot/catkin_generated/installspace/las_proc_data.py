#!/usr/bin/env python3

import rospy
import std_msgs
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import sys
from move_robot.msg import las_mes
from math import *

SCRIPTS_PATH = '/home/vivek/catkin_ws/src/move_robot/scripts'
sys.path.insert(0, SCRIPTS_PATH)


class Laser_Mes:

    def __init__(self, name):
        self.name = name
        self.laser_publisher = rospy.Publisher(f'{name}las_mes', las_mes, queue_size=0)
        self.odom_subscriber = rospy.Subscriber(f'{name}odom', Odometry, self.odom_callback)
        self.scan_subscriber = rospy.Subscriber(f'{name}scan', LaserScan, self.scan_callback)
        self.rate = rospy.Rate(50) # 10hz
        self.current_odom = None
        self.distances = np.array([])
        self.angles = np.array([])
        self.information = np.array([])
        self.max_range = None
        self.yaw = None


    def odom_callback(self, data):
        # Update the current position of the turtlebot
        self.current_odom = data
        self.odom_x = self.current_odom.pose.pose.position.x
        self.odom_y = self.current_odom.pose.pose.position.y
        self.ornt_q = data.pose.pose.orientation
        ornt_list = [self.ornt_q.x, self.ornt_q.y, self.ornt_q.z, self.ornt_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(ornt_list)

        if self.yaw < 0:
            self.yaw = 2 * np.pi + self.yaw  # 0->360 degrees >> 0->2pi


    def scan_callback(self, msgScan):
        
        self.max_range = msgScan.range_max
        self.distances_ = np.array([])
        self.angles_ = np.array([])
        self.information_ = np.array([])

        for i in range(len(msgScan.ranges)):
            # angle calculation
            ang = i * msgScan.angle_increment # 1 degree resolution for 360 degree

            # distance calculation. Default values of 'ranges' is : [inf, inf, inf, inf, inf, ....]
            if ( msgScan.ranges[i] > msgScan.range_max ):
                dist = msgScan.range_max #range_max is 3.5m
            elif ( msgScan.ranges[i] < msgScan.range_min ):
                dist = msgScan.range_min #range_min is 11cm
            else:
                dist = msgScan.ranges[i]

            # smaller the distance, bigger the information (measurement is more confident)
            inf = ((msgScan.range_max - dist) / msgScan.range_max) ** 2 

            self.distances_ = np.append(self.distances_, dist)
            self.angles_ = np.append(self.angles_, ang)
            self.information_ = np.append(self.information_, inf)
            
        self.distances = self.distances_ 
        self.angles = self.angles_
        self.information = self.information_

    def lidar_scan_xy(self):
        """
        Lidar measurements in X-Y plane
        """
        # print("entered")
        # print(self.distances)
        self.distances_x = np.array([])
        self.distances_y = np.array([])

        for (dist, ang) in zip(self.distances, self.angles):
            self.distances_x = np.append(self.distances_x, dist * np.cos(ang + self.yaw))
            self.distances_y = np.append(self.distances_y, dist * np.sin(ang + self.yaw))


    def get_laser_data(self,robot_namespace):
        isFirstLoop = True
        while not rospy.is_shutdown():
            self.lidar_scan_xy()
            d = [self.distances, self.distances_x, self.distances_y, self.angles+self.yaw]
            d = [[float(x) for x in d_sublist] for d_sublist in d]

            # Create a Float64MultiArray message with the values
            lm = las_mes()
            lm.las_dist = d[0]
            lm.las_dist_x = d[1]
            lm.las_dist_y = d[2]
            lm.las_angles = d[3]
            # print(lm)
            self.laser_publisher.publish(lm)
            # rospy.loginfo("Laser measurements of the agent are published")
            self.rate.sleep()

            if isFirstLoop:
                rospy.set_param(robot_namespace+'las_proc_data_published', True)
                isFirstLoop = False


if __name__ == "__main__":
    try:
        
        rospy.init_node('laser_publisher', anonymous=True)

        # Get the namespace parameter from the launch file
        # robot_namespace = rospy.get_param('~robot_namespace', 'tb3_0')
        robot_namespace = rospy.get_namespace()

        pass_rate = rospy.Rate(30)

        while not rospy.has_param(robot_namespace+'set_initial_goal'):  # untill set_inital_goal is set tb_las_mes is not constructed
            pass_rate.sleep()

        
        if rospy.has_param(robot_namespace+'set_initial_goal'):
            rospy.loginfo('set_initial_goal parameter has been set')
        else:
            rospy.logwarn('set_initial_goal parameter is not set')

        # Create a turtlebot laser object and get the laser data
        tb_las_mes = Laser_Mes(robot_namespace)
        tb_las_mes.get_laser_data(robot_namespace)
    
    except rospy.ROSInterruptException:
        pass