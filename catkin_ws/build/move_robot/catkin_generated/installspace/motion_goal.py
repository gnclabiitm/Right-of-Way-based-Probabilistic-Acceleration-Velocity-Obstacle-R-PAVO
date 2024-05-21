#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

current_odom = None 
cmd_vel_pub = None
roll = pitch = yaw = 0.0
kp = 0.25

#Function to assign the current odometry messages that are sunscribed 
def odomCallback(msg):
    # rospy.loginfo("Entered Callback /n")
    global current_odom
    global roll, pitch, yaw
    current_odom = msg
    ornt_q = msg.pose.pose.orientation
    ornt_list = [ornt_q.x, ornt_q.y, ornt_q.z, ornt_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(ornt_list)
    #rospy.loginfo(current_odom.pose.pose.position.x)




#Function to move the turtlebot to its goal position
def move_turtlebot_to_position(goal_x, goal_y, goal_tol):
    rospy.loginfo("Entered move_turtlebot_to_position /n")
    global current_odom
    global cmd_vel_pub
    global yaw, kp
    # rospy.loginfo(current_odom.pose.pose.position.x)

    #Set the maximum velocities (from turtlebot3 data)
    max_linear_vel = 0.2
    max_angular_vel = 0.5

    #Set the initial velocities
    linear_vel = 0.05
    angular_vel = 0.1

    #Loop to run the robot towards the goal untill it reaches or if terminal shutdown is done
    while not rospy.is_shutdown():

        #Calculating the distance and angle between the robot's current and goal position
        if current_odom is not None:
            rospy.loginfo("if activated \n")
            dx = goal_x - current_odom.pose.pose.position.x
            dy = goal_y - current_odom.pose.pose.position.y 
            distance = math.sqrt(dx**2 + dy**2)
            theta = math.atan2(dy,dx)
            angle = (theta - yaw)

            #Calculating the linear and angular velocities based on the calculated distance and angle
            #linear_vel = min(distance, max_linear_vel)
            #angular_vel = min(angle, max_angular_vel)
            
            if angle>0.1:
                angular_vel = kp*angle
                linear_vel = 0.1 
            elif angle<-0.1:
                angular_vel = -kp*angle 
                linear_vel = 0.1 
            else:
                angular_vel = 0.0
                linear_vel = 0.22

            #Check if the goal is reached (within the tolerance range)
            #come out of the loop and send zero velocities
            if distance < goal_tol:
                rospy.loginfo("Goal position reached")
                break 

            if rospy.is_shutdown():
                break

        #Creating a twist message to set the velocities for the later publishment
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel  #since directed towards goal
        twist_msg.angular.z = angular_vel  #since only yaw rotation (along z) is possible

        #Publishing the twist messages to the '\cmd_vel' topic
        cmd_vel_pub.publish(twist_msg)

        rospy.sleep(0.5)
    
    #stop the robot by publishing zero velocities to /cmd_vel 
    twist_msg = Twist()
    cmd_vel_pub.publish(twist_msg) #default Twist will give zero velocities


if __name__ == '__main__':
    try:

        #Initialising the node
        rospy.init_node('turtlebot3_move_to_goal', anonymous=True)

        #Subscribing to the topic '/odom' (Odometry msg_type) to get the current position of the robot in Inertial frame. 
        #Callback function is to assign the pose values
        rospy.Subscriber('tb3_1/odom', Odometry, odomCallback)
        #rospy.loginfo(current_odom)

        #Publishing the linear and angular velocities to the topic '/cmd_vel' (Twist msg_type) to move the robot towards goal (10 times per second : queue_size)
        cmd_vel_pub = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=10)   

        #set the goal position
        goal_x = 1.0
        goal_y = 1.0
        goal_tol = 0.1

        #Callback function to move the turtlebot to the goal
        move_turtlebot_to_position(goal_x, goal_y, goal_tol)

    #Interruption exceptions occured due to sleep() functions or any other
    except rospy.ROSInterruptException:
        pass
	    #rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
        