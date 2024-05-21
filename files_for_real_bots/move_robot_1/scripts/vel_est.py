#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import numpy as np

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector')
        
        # Parameters
        self.cluster_distance_threshold = rospy.get_param('~cluster_distance_threshold', 0.2) # maximum distance between two points to be considered part of the same cluster
        self.cluster_size_threshold = rospy.get_param('~cluster_size_threshold', 3) # minimum number of points in a cluster
        self.cluster_max_time_diff = rospy.get_param('~cluster_max_time_diff', 0.1) # maximum time difference between two points to be considered part of the same cluster
        
        # State variables
        self.last_scan = None
        self.last_scan_time = None
        
        # Publishers
        self.object_pos_pub = rospy.Publisher('object_position', PointStamped, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('scan', LaserScan, self.laser_scan_callback)
        
    def laser_scan_callback(self, msg):
        # Save the latest scan and time
        self.last_scan = msg
        self.last_scan_time = rospy.Time.now()
        
        # Cluster the laser scan data
        clusters = self.cluster_scan(self.last_scan)
        
        # Calculate the velocities of each cluster
        velocities = self.calculate_velocities(clusters, self.last_scan_time)
        
        # Publish the position of each object and its velocity
        for i, cluster in enumerate(clusters):
            position = self.calculate_cluster_position(cluster)
            velocity = velocities[i]
            
            msg = PointStamped()
            msg.header.frame_id = self.last_scan.header.frame_id
            msg.header.stamp = self.last_scan_time
            msg.point.x = position[0]
            msg.point.y = position[1]
            msg.point.z = position[2]
            self.object_pos_pub.publish(msg)
            
            rospy.loginfo("Object at position (%f, %f, %f) moving at velocity %f", position[0], position[1], position[2], velocity)
        
    def cluster_scan(self, scan):
        ranges = np.array(scan.ranges)
        
        # Convert scan to cartesian coordinates
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        points = np.zeros((len(scan.ranges), 2))
        points[:,0] = np.cos(angles) * ranges
        points[:,1] = np.sin(angles) * ranges
        
        # Cluster the points using a simple distance-based algorithm
        clusters = []
        while points.shape[0] > 0:
            current_point = points[0,:]
            cluster_mask = np.linalg.norm(points - current_point, axis=1) < self.cluster_distance_threshold
            
            if np.sum(cluster_mask) >= self.cluster_size_threshold:
                clusters.append(points[cluster_mask,:])
            
            points = points[~cluster_mask,:]
        
        return clusters
        
    def calculate_cluster_position(self, cluster):
        x = np.mean(cluster[:,0])
        y = np.mean(cluster[:,1])
        z = np.mean(cluster[:,2])
        return [x, y, z]
    
    def calculate_velocities(self, clusters, scan_time):
        velocities = []
        
        for cluster in clusters:
            # Get the time stamp of the first and last points in the cluster
            t1 = rospy.Time.from_sec(cluster[0,2])
            t2 = rospy.Time.from_sec(cluster[-1,2])
            
            # Calculate the time difference

            time_diff = (t2 - t1).to_sec()
            
            # Calculate the distance traveled by the cluster
            delta_x = cluster[-1,0] - cluster[0,0]
            delta_y = cluster[-1,1] - cluster[0,1]
            delta_z = cluster[-1,2] - cluster[0,2]
            distance = np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
            
            # Calculate the velocity of the cluster
            velocity = distance / time_diff
            
            velocities.append(velocity)
            
        return velocities

if __name__ == '__main__':
    detector = ObjectDetector()
    rospy.spin()
