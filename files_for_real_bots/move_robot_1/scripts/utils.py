#!/usr/bin/env python

#from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import *

import numpy as np

def lidar_scan(msgScan):
    """
    Convert LaserScan msg to array
    """
    distances = np.array([])
    angles = np.array([])
    information = np.array([])

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

        distances = np.append(distances, dist)
        angles = np.append(angles, ang)
        information = np.append(information, inf)

    # distances in [m], angles in [radians], information [0-1]
    return ( distances, angles, information )


def lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom):
	"""
	Lidar measurements in X-Y plane
	"""
	distances_x = np.array([])
	distances_y = np.array([])

	for (dist, ang) in zip(distances, angles):
		distances_x = np.append(distances_x, dist * np.cos(ang + theta_odom))
		distances_y = np.append(distances_y, dist * np.sin(ang + theta_odom))

	return (distances_x, distances_y)


def transform_orientation(orientation_q):
    """
    Transform theta to [radians] from [quaternion orientation]
    """
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    if yaw < 0:
        yaw = 2 * np.pi + yaw  # 0->360 degrees >> 0->2pi
    return yaw


def get_odom_orientation(msgOdom):
    """"
    Get theta from Odometry msg in [radians]
    """
    orientation_q = msgOdom.pose.pose.orientation
    theta = transform_orientation(orientation_q)
    return theta
    

def get_odom_position(msgOdom):
    """
    Get (x,y) coordinates from Odometry msg in [m]
    """
    x = msgOdom.pose.pose.position.x
    y = msgOdom.pose.pose.position.y
    return (x, y)

def getx2y2(theta,l,w):
    """
    Get (x2,y2) coordinates for the grid map to find the occlusion cells using line iterator 
    A = odom_x, B = odom_y
    l = L/2, w = W/2
    m = atan2(temp_position[1])
    """
    if (0<=theta<pi/4):
        x2 = w
        y2 = x2*tan(theta)
    elif (pi/4<=theta<pi/2):
        y2 = l
        x2 = y2/tan(theta)
    elif (pi/2<=theta<3*pi/4):
        y2 = l
        x2 = y2/tan(theta)
    elif (3*pi/4<=theta<pi):
        x2 = -w
        y2 = x2*tan(theta)
    elif (pi<=theta<5*pi/4):
        x2 = -w
        y2 = x2*tan(theta)
    elif (5*pi/4<=theta<6*pi/4):
        y2 = -l
        x2 = y2/tan(theta)
    elif (6*pi/4<=theta<7*pi/4):
        y2 = -l
        x2 = y2/tan(theta)
    elif (7*pi/4<=theta<=8*pi/4):
        x2 = w
        y2 = x2*tan(theta)
    
    return x2,y2



    # if (m>=1) :
        
    #     if (x1>A) and (y1>B): # Quadrant-I 
    #         print(A,B)
    #         y2 = B+l
    #         x2 = (y2 - c1)/m
    #         return [x2,y2]
    #     if (x1<A) and (y1>B): # Quadrant-III
    #         y2 = -B
    #         x2 = (y2 - c1)/m
    #         return [x2,y2]
    # if (0<=m<1) :
    #     if (x1>A) and (y1<=B): # Quadrant-I
    #         x2 = A
    #         y2 = m*x2 + c1
    #         return [x2,y2]
    #     if (x1<=A) and (y1>B): # Quadrant-III
    #         x2 = -A
    #         y2 = m*x2 + c1
    #         return [x2,y2]
    # if (0>m>=-1) :
    #     if (x1<=A) and (y1<=B): # Quadrant-II
    #         y2 = B
    #         x2 = (y2 - c1)/m
    #         return [x2,y2]
    #     if (x1>A) and (y1>B): # Quadrant-IV
    #         y2 = -B
    #         x2 = (y2 - c1)/m
    #         return [x2,y2]
    # if (m<-1) :
    #     if (x1<=A) and (y1<=B): # Quadrant-II
    #         x2 = -A
    #         y2 = m*x2 + c1
    #         return [x2,y2]
    #     if (x1>A) and (y1>B): # Quadrant-IV
    #         x2 = A
    #         y2 = m*x2 + c1
    #         return [x2,y2]
    # print(x2,y2)
    

    
def set_pixel_color(bgr_image, x, y, color):
    """
    Set 'color' to the given pixel (x,y) on 'bgr_image'
    """

    if x < 0 or y < 0 or x >= bgr_image.shape[0] or y >= bgr_image.shape[1]:
        return 

    if color == 'BLUE':

        bgr_image[x, y, 0] = 1.0
        bgr_image[x, y, 1] = 0.0
        bgr_image[x, y, 2] = 0.0

    elif color == 'GREEN':

        bgr_image[x, y, 0] = 0.0
        bgr_image[x, y, 1] = 1.0
        bgr_image[x, y, 2] = 0.0

    elif color == 'RED':

        bgr_image[x, y, 0] = 0.0
        bgr_image[x, y, 1] = 0.0
        bgr_image[x, y, 2] = 1.0