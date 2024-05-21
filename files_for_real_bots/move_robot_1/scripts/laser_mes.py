#!/usr/bin/env python

import rospy
import std_msgs
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from utils import lidar_scan, get_odom_position, get_odom_orientation, lidar_scan_xy, getx2y2
#import utils
import sys
from move_robot.msg import las_mes
from move_robot.srv import xycoord, xycoordResponse
from math import *

SCRIPTS_PATH = '/home/ubuntu/catkin_ws/src/move_robot/scripts'
sys.path.insert(0, SCRIPTS_PATH)


def coordCallback(req):
    theta = req.occ_theta
    l = req.l
    w = req.w

    # try:
    #     getx2y2(theta,l,w)
    # except:
    #     print("func failed")
    if (0*pi/4<=theta<1*pi/4): #1
        x2 = w
        y2 = x2*tan(theta)
    elif (1*pi/4<=theta<2*pi/4): #2
        y2 = l
        x2 = y2/tan(theta)
    elif (2*pi/4<=theta<3*pi/4): #3
        y2 = l
        x2 = y2/tan(theta)
    elif (3*pi/4<=theta<pi): #4
        x2 = -w
        y2 = x2*tan(theta)
    elif (-pi<=theta<-3*pi/4): #5
        x2 = -w
        y2 = x2*tan(theta)
    elif (-3*pi/4<=theta<-2*pi/4):#6
        y2 = -l
        x2 = y2/tan(theta)
    elif (-2*pi/4<=theta<-1*pi/4): #7
        y2 = -l
        x2 = y2/tan(theta)
    elif (-1*pi/4<=theta<=-0*pi/4):
        x2 = w
        y2 = x2*tan(theta)
    else:
        x2 = w
        y2 = l
    
    # return x2,y2
    # print("theta, x2, y2 are: ",theta, x2, y2)
    # print("theta, l, w are: ",theta, l, w)
    return xycoordResponse(x2,y2)


# prev_time = None
# prev_dists = []
# prev_velocities = []

# def VelFinder(distances,prev_time,prev_dists,prev_velocities,scan_time):
#     # global prev_time, prev_dists, prev_velocities
#     if prev_time == None:
#         prev_time = rospy.Time.now()
                
#     curr_time = rospy.Time.now() 
#     elap_time = (curr_time.secs - prev_time.secs)
#     # prev_time = curr_time

#     curr_dists = []
#     velocities = []

#     for i in range(len(distances)):

#         if len(prev_dists) == 0:
#             prev_dists = distances
        
#         curr_dists.append(distances[i])
#         print(distances[i],curr_dists[i])
#         print(curr_time.secs,prev_time.secs)
#         velocities.append((curr_dists[i]-prev_dists[i])/elap_time)
#         print(velocities[i],i, elap_time)
#     # prev_dists = curr_dists

#     return velocities, curr_time




if __name__ == "__main__":
    try:
        rospy.init_node('laser_publisher', anonymous=True)

        prev_time = None
        prev_dists = []
        prev_velocities = []
        n = 5

        las_topic = "las_mes"
        las_pub = "las_pub"
        for i in range(n):		
            las_topic = las_topic.replace(las_topic,"las_mes"+"_"+str(i))
            # las_pub = las_pub.replace(las_pub,"laser_pub"+"_"+str(i))
            if i == 0:
                laser_pub_0 = rospy.Publisher(las_topic, las_mes, queue_size=10)
            elif i == 1:
                laser_pub_1 = rospy.Publisher(las_topic, las_mes, queue_size=10)
            elif i == 2:
                laser_pub_2 = rospy.Publisher(las_topic, las_mes, queue_size=10)
            elif i == 3:
                laser_pub_3 = rospy.Publisher(las_topic, las_mes, queue_size=10)
            elif i == 4:
                laser_pub_4 = rospy.Publisher(las_topic, las_mes, queue_size=10)

            # laser_pub = las_pub
            # laser_pub = rospy.Publisher(las_topic, las_mes, queue_size=10)
        # laser_mes = std_msgs.msg.Float64MultiArray()
        # LidarMeasurement(laser_mes)

        # Set the rate of publishing
        rate = rospy.Rate(30)
        occlu_service = rospy.Service("occlu_xy", xycoord, coordCallback)


        # occlu_service.shutdown()
        # Publish the values
        

        while not rospy.is_shutdown():
            lm = [[],[],[],[],[]]
            # d = [[],[],[],[],[]]
            turtle_ids = ['tb3_0','tb3_1','tb3_2','tb3_3','tb3_4']
            tb3_scan = 'tb3_scan'
            tb3_odom = 'tb3_odom'

            # Set the list of lists of values to publish
            las_pub = "las_pub"
            for i in range(n):
                tb3_scan = tb3_scan.replace(tb3_scan,turtle_ids[i]+'/scan')
                msgScan = rospy.wait_for_message(tb3_scan, LaserScan) #will provide only one message (laser scan) 
                distances, angles, information = lidar_scan(msgScan)  # distances in [m], angles in [radians]

                # Odometry measurements
                tb3_odom = tb3_odom.replace(tb3_odom,turtle_ids[i]+'/odom')
                msgOdom = rospy.wait_for_message(tb3_odom, Odometry) #will provide only one message (odometry data)
                x_odom, y_odom = get_odom_position(msgOdom)   # x,y in [m]
                theta_odom = get_odom_orientation(msgOdom)    # theta in [radians]
                print("data collected from /scan and /odom topics")
                # Lidar measurements in X-Y plane (measurements in odom frame)
                distances_x, distances_y = lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom)
                max_range = msgScan.range_max

                # velocities, curr_time = VelFinder(distances,prev_time,prev_dists,prev_velocities, msgScan.scan_time)
                # prev_time = curr_time
                # prev_dists = distances
                # prev_velocities = velocities

                # lm = laser_mes
                d = [distances, distances_x, distances_y, angles+theta_odom]#, velocities]
                d = [[float(x) for x in d_sublist] for d_sublist in d]

                # Create a Float64MultiArray message with the values
                
                lm[i] = las_mes()
                lm[i].las_dist = d[0]
                lm[i].las_dist_x = d[1]
                lm[i].las_dist_y = d[2]
                lm[i].las_angles = d[3]
                # lm.velocities = d[4]
            
                # Publish the message
                if i == 0:
                    laser_pub_0.publish(lm[i])
                elif i == 1:
                    laser_pub_1.publish(lm[i])
                elif i == 2:
                    laser_pub_2.publish(lm[i])
                elif i == 3:
                    laser_pub_3.publish(lm[i])
                elif i == 4:
                    laser_pub_4.publish(lm[i])
                # laser_pub.publish(lm[i])
                # print(laser_pub)
                print("calculated laser measurements data for tb3_",i, "has been published")
            # print(lm)
            # Sleep for the specified rate

            # service to send the requested coordinates
            

            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass
