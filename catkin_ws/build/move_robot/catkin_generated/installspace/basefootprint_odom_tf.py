#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# class TF_broadcast_2:
#     def __init__(self, name, t):
#         self.name = name
#         t.header.frame_id = f'{name}odom'
#         t.child_frame_id = f'{name[:-1]}_tf/odom'
#         self.rate2 = rospy.Rate(30.0)


#     def tf_broadcast2(self,t2, br2):
#         while not self.odomCallbackcalled:
#             self.rate2.sleep()
            
#         while not rospy.is_shutdown():
#             # print("entered while loop")
#             # print(self.odom_x)
#             t2.header.stamp = rospy.Time.now()
#             t2.transform.translation.x = 0.0
#             t2.transform.translation.y = 0.0
#             t2.transform.translation.z = 0.0
#             t2.transform.rotation.x = 0.0
#             t2.transform.rotation.y = 0.0
#             t2.transform.rotation.z = 0.0
#             t2.transform.rotation.w = 0.0

#             br2.sendTransform(t2)

#             # print(t)
#             self.rate2.sleep()

class TF_broadcast:
    def __init__(self, name, t):
        self.name = name
        t.header.frame_id = "odom"
        t.child_frame_id = f'{name}base_footprint'
        # print(t)
        self.odom_subscriber = rospy.Subscriber(f'{name}odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(30.0)
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
            
        while not rospy.is_shutdown():
            # print("entered while loop")
            # print(self.odom_x)
            t.header.stamp = rospy.Time.now()
            t.transform.translation.x = self.odom_x
            t.transform.translation.y = self.odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = self.ornt_q.x
            t.transform.rotation.y = self.ornt_q.y
            t.transform.rotation.z = self.ornt_q.z
            t.transform.rotation.w = self.ornt_q.w

            br.sendTransform(t)

            # print(t)
            self.rate.sleep()
        

if __name__ == '__main__':
    rospy.init_node('bf_odom_tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # br2 = tf2_ros.TransformBroadcaster()
    # t2 = geometry_msgs.msg.TransformStamped()

    robot_namespace = rospy.get_namespace()
    # robot_namespace = rospy.get_param('robot_namespace', 'tb3_0')

    pass_rate = rospy.Rate(30)
    while not rospy.has_param(robot_namespace+'set_initial_goal'):
        pass_rate.sleep()

    tf_agent = TF_broadcast(robot_namespace, t)
    tf_agent.tf_broadcast(t, br)

    # tf_agent = TF_broadcast_2(robot_namespace, t2)
    # tf_agent.tf_broadcast2(t2, br2)