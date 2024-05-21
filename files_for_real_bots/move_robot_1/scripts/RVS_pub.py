#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# Define the ROS node
rospy.init_node('next_reachable_velocity_points')

# Define the ROS publisher for the next reachable velocity points
points_pub = rospy.Publisher('next_reachable_velocity_points', Marker, queue_size=1)

# Define the Marker message for the next reachable velocity points
points_marker = Marker()
points_marker.header.frame_id = "odom" # Change the frame_id if necessary
points_marker.type = Marker.POINTS
points_marker.action = Marker.ADD
points_marker.scale.x = 0.1 # Change the size of the points if necessary
points_marker.scale.y = 0.1
points_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0) # Change the color of the points if necessary

# Define the array of x and y velocity points
x_points = [1.0, 2.0, 3.0, 4.0, 5.0] # Replace with your actual x velocity points
y_points = [2.0, 4.0, 6.0, 8.0, 10.0] # Replace with your actual y velocity points

# Define an empty array for the next reachable velocity points
points = []

# Loop through the x and y points and create a new Point message for each point
for i in range(len(x_points)):
    point = Point()
    point.x = x_points[i]
    point.y = y_points[i]
    point.z = 0.0 # Set the z-coordinate to 0.0 (assuming a 2D space)
    points.append(point)


# Loop through the next reachable velocity points and add them to the Marker message
for point in points:
    points_marker.points.append(point)

# Loop rate for publishing the Marker message
loop_rate = rospy.Rate(10)

# Publish the Marker message at the defined loop rate
while not rospy.is_shutdown():
    points_pub.publish(points_marker)
    loop_rate.sleep()
