#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math

def callback(data):
    x_vel = data.linear.x
    y_vel = data.linear.y
    ang_vel = data.angular.z
    print(data)

    # Convert to velocity along x and y
    vx = x_vel * math.cos(ang_vel) - y_vel * math.sin(ang_vel)
    vy = x_vel * math.sin(ang_vel) + y_vel * math.cos(ang_vel)
    print("x_vel ", x_vel)
    print("y_vel ", y_vel)
    # Do something with vx and vy
    print("Velocity along x:", vx)
    print("Velocity along y:", vy)

def listener(name):
    rospy.init_node('velocity_converter', anonymous=True)
    rospy.Subscriber(f'{name}/cmd_vel', Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    # robot_namespace = rospy.get_namespace()
    robot_namespace = "tb3_0"
    listener(robot_namespace)
