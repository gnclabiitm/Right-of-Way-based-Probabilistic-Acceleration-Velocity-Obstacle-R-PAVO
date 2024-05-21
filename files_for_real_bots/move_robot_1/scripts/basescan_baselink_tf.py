#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class TF_broadcast:
    def __init__(self, name, t_scan, tf_buffer):
        self.name = name
        t_scan.header.frame_id = f'{name[1:]}base_link'
        self.tb_bs = f'{name[1:]}base_scan'
        t_scan.child_frame_id = f'{name[1:]}new_base_scan'
        print(t_scan.header.frame_id,  self.tb_bs, t_scan.child_frame_id)
        self.laser_sub = rospy.Subscriber(f'{name}scan', LaserScan, self.lasercallback)
        self.laser_pub = rospy.Publisher('scan_new', LaserScan, queue_size=10)

        # Lookup the transform between the base_scan frame and the odom frame
        try:
            trans = tf_buffer.lookup_transform(t_scan.header.frame_id ,self.tb_bs, rospy.Time(0), rospy.Duration(15.0))
            # Extract the position and orientation data from the transform
            self.pose = geometry_msgs.msg.PoseStamped()
            self.pose.header = trans.header
            self.pose.pose.position = trans.transform.translation
            self.pose.pose.orientation = trans.transform.rotation
        except tf2_ros.LookupException:
            rospy.loginfo("transform sikkilla1")
            pass


        self.rate = rospy.Rate(30.0)


    def lasercallback(self,scan):

        # Lookup the transform between the base_scan frame and the odom frame
        try:
            trans = tf_buffer.lookup_transform(t_scan.header.frame_id ,self.tb_bs,rospy.Time(0), rospy.Duration(5.0))
            self.pose = geometry_msgs.msg.PoseStamped()
            self.pose.header = trans.header
            self.pose.pose.position = trans.transform.translation
            self.pose.pose.orientation = trans.transform.rotation
        except tf2_ros.LookupException:
            rospy.loginfo("transform sikkilla2")
            pass

        # Change the frame ID of the laser scan message
        scan.header.frame_id = f'{self.name[1:]}new_base_scan'
        

        # Publish the modified laser scan message
        self.laser_pub.publish(scan)

    def tf_broadcast(self, br_scan, t_scan):
        # while not self.odomCallbackcalled:
        #     self.rate.sleep()
            
        while not rospy.is_shutdown():            
            t_scan.header.stamp = rospy.Time.now()
            t_scan.transform.translation = self.pose.pose.position
            t_scan.transform.rotation = self.pose.pose.orientation

            br_scan.sendTransform(t_scan)

            # print(t)
            self.rate.sleep()
        

if __name__ == '__main__':
    rospy.init_node('bf_bl_tf_broadcaster')
    br_scan = tf2_ros.TransformBroadcaster()
    t_scan = geometry_msgs.msg.TransformStamped()

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    robot_namespace = rospy.get_namespace()
    # robot_namespace = rospy.get_param('robot_namespace', 'tb3_0')

   # pass_rate = rospy.Rate(30)
   # while not rospy.has_param(robot_namespace+'set_initial_goal'):
   #     pass_rate.sleep()

    tf_agent = TF_broadcast(robot_namespace, t_scan, tf_buffer)
    tf_agent.tf_broadcast(br_scan, t_scan)
    
    
