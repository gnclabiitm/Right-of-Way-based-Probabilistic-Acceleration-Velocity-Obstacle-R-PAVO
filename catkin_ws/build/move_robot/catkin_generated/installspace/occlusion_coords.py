#!/usr/bin/env python3

import rospy
import sys
from math import *
from move_robot.srv import xycoord, xycoordResponse

SCRIPTS_PATH = '/home/vivek/catkin_ws/src/move_robot/scripts'
sys.path.insert(0, SCRIPTS_PATH)

def coordCallback(req):
    theta = req.occ_theta
    l = req.l
    w = req.w
    # print("occlusion callback is called")

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

    return xycoordResponse(x2,y2)

if __name__ == "__main__":
    try:
        rospy.init_node('Occlusion_coord_finder', anonymous=True) 
        # Get the namespace parameter from the launch file
        # robot_namespace = rospy.get_param('~robot_namespace', 'tb3_0')
        robot_namespace = rospy.get_namespace()
        
        pass_rate = rospy.Rate(30)
        while not rospy.has_param(robot_namespace+'set_initial_goal'):
            # rospy.loginfo('waiting for set_initial_goal param')
            pass_rate.sleep()

        
        if rospy.has_param(robot_namespace+'set_initial_goal'):
            rospy.loginfo('set_initial_goal parameter has been set')
        else:
            rospy.logwarn('set_initial_goal parameter is not set')

        occlu_service = rospy.Service(robot_namespace+'occlu_xy', xycoord, coordCallback)
        rospy.set_param(robot_namespace+'service_server_created', True)

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():    
            rate.sleep()             
           
    except rospy.ROSInterruptException:
        pass