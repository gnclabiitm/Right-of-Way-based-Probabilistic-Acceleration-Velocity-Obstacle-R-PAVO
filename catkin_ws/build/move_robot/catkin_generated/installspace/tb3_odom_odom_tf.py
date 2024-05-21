#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler
import math

class TF_broadcast:
    def __init__(self, name, t, initial_x,initial_y, initial_z, initial_yaw):
        self.name = name
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.initial_z = initial_z
        self.initial_yaw = initial_yaw
        t.header.frame_id = "odom"
        t.child_frame_id = f'{name}odom'
        # print(t)
        self.odom_subscriber = rospy.Subscriber(f'{name}odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10.0)
        self.odomCallbackcalled = False


    def odom_callback(self, data):
        # Update the current position of the turtlebot
        # print("odom is calledback")
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y
        # self.odom_z = data.pose.pose.position.z
        self.ornt_q = data.pose.pose.orientation
        self.ornt_list = [self.ornt_q.x, self.ornt_q.y, self.ornt_q.z, self.ornt_q.w]
        # (self.roll, self.pitch, self.yaw) = euler_from_quaternion(ornt_list)
        self.odomCallbackcalled = True

    def tf_broadcast(self,t, br):
        while not self.odomCallbackcalled:
            self.rate.sleep()
            
        q = quaternion_from_euler(0, 0, self.initial_yaw)
        while not rospy.is_shutdown():
            # print("entered while loop")
            # print(self.odom_x)
            
            t.header.stamp = rospy.Time.now()
            t.transform.translation.x = self.initial_x
            t.transform.translation.y = self.initial_y
            t.transform.translation.z = self.initial_z
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)

            # print(t)
            self.rate.sleep()
        

if __name__ == '__main__':
    rospy.init_node('tb3_odom_odom_tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # br2 = tf2_ros.TransformBroadcaster()
    # t2 = geometry_msgs.msg.TransformStamped()

    robot_namespace = rospy.get_namespace()
    # robot_namespace = rospy.get_param('robot_namespace', 'tb3_0')
    initial_x = rospy.get_param('~initial_x', 0.0)
    initial_y = rospy.get_param('~initial_y', 0.0)
    initial_z = rospy.get_param('~initial_z', 0.0)
    initial_yaw = rospy.get_param('~initial_yaw', 1.57)

    # pass_rate = rospy.Rate(30)
    # while not rospy.has_param(robot_namespace+'set_initial_goal'):
    #     pass_rate.sleep()

    tf_agent = TF_broadcast(robot_namespace, t, initial_x,initial_y, initial_z, initial_yaw)
    tf_agent.tf_broadcast(t, br)

    # tf_agent = TF_broadcast_2(robot_namespace, t2)
    # tf_agent.tf_broadcast2(t2, br2)
    
    