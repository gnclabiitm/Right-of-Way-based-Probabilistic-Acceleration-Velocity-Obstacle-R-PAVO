#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

def set_initial_goal(robot_namespace, initial_x, initial_y, initial_z, initial_yaw, goal_x, goal_y, goal_z):
    # Set the initial position for the robot
    initial_pose_pub = rospy.Publisher(robot_namespace + 'initialpose', PoseWithCovarianceStamped, queue_size=10)
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = 'odom'
    initial_pose.pose.pose.position.x = initial_x
    initial_pose.pose.pose.position.y = initial_y
    initial_pose.pose.pose.position.z = initial_z
    initial_pose.pose.pose.orientation.w = initial_yaw
    initial_pose_pub.publish(initial_pose)
    rospy.loginfo('Set initial position for %s: (%s, %s, %s, %s)', robot_namespace, initial_x, initial_y, initial_z, initial_yaw)

    # Set the goal position for the robot
    goal_pose_pub = rospy.Publisher(robot_namespace + 'goal', PoseStamped, queue_size=10)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'odom'
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.position.z = goal_z
    # goal_pose.pose.orientation.w = 1.0
    goal_pose_pub.publish(goal_pose)
    rospy.loginfo('Set goal position for %s: (%s, %s, %s)', robot_namespace, goal_x, goal_y, goal_z)

    rospy.set_param(robot_namespace+'set_initial_goal', True)

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

    rate = rospy.Rate(10)
    while rospy.is_shutdown():
        initial_marker = Marker()
        initial_marker.header.frame_id = "odom" # Change the frame_id if necessary
        initial_marker.header.stamp = rospy.Time.now()
        initial_marker.ns = "markers"
        initial_marker.id = 0
        initial_marker.type = Marker.CUBE
        initial_marker.action = Marker.ADD
        initial_marker.scale.x = 0.2 # Change the size of the points if necessary
        initial_marker.scale.y = 0.2
        initial_marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.0) # Change the color of the points if necessary
        initial_marker.pose.position.x = initial_x
        initial_marker.pose.position.y = initial_y
        initial_marker.pose.position.z = initial_z
        initial_marker.pose.orientation.x = 0.0
        initial_marker.pose.orientation.y = 0.0
        initial_marker.pose.orientation.z = 0.0
        initial_marker.pose.orientation.w = 1.0


        # goal_pub = rospy.Publisher('goal_points', Marker, queue_size=1)
        goal_marker = Marker()
        goal_marker.header.frame_id = "odom" # Change the frame_id if necessary
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.ns = "markers"
        goal_marker.id = 1
        goal_marker.type = Marker.CYLINDER
        goal_marker.action = Marker.ADD
        goal_marker.scale.x = 0.2 # Change the size of the points if necessary
        goal_marker.scale.y = 0.2
        goal_marker.color = ColorRGBA(1.0, 0.0, 1.0, 0.0) # Change the color of the points if necessary
        goal_marker.pose.position.x = goal_x
        goal_marker.pose.position.y = goal_y
        goal_marker.pose.position.z = goal_z
        goal_marker.pose.orientation.x = 0.0
        goal_marker.pose.orientation.y = 0.0
        goal_marker.pose.orientation.z = 0.0
        goal_marker.pose.orientation.w = 1.0

    
        marker_pub.publish(initial_marker)
        marker_pub.publish(goal_marker)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('set_initial_goal')
    robot_namespace = rospy.get_namespace()

    rospy.wait_for_message(robot_namespace+'odom', Odometry)
    rospy.wait_for_message(robot_namespace+'scan', LaserScan)

    initial_x = rospy.get_param('~initial_x', 0.0)
    initial_y = rospy.get_param('~initial_y', 0.0)
    initial_z = rospy.get_param('~initial_z', 0.0)
    initial_yaw = rospy.get_param('~initial_yaw', 1.57)
    goal_x = rospy.get_param('~goal_x', 1.0)
    goal_y = rospy.get_param('~goal_y', 1.0)
    goal_z = rospy.get_param('~goal_z', 0.0)
    set_initial_goal(robot_namespace, initial_x, initial_y, initial_z, initial_yaw, goal_x, goal_y, goal_z)
    rospy.set_param(robot_namespace+'set_initial_goal', True)
   # rospy.spin()

