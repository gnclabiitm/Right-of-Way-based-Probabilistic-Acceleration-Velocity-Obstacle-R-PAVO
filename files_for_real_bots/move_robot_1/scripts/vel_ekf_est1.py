#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector')
        
        # Parameters
        self.cluster_distance_threshold = rospy.get_param('~cluster_distance_threshold', 0.2) # maximum distance between two points to be considered part of the same cluster
        self.cluster_size_threshold = rospy.get_param('~cluster_size_threshold', 3) # minimum number of points in a cluster
        self.cluster_max_time_diff = rospy.get_param('~cluster_max_time_diff', 0.1) # maximum time difference between two points to be considered part of the same cluster
        
        self.ekf_process_variance = rospy.get_param('~ekf_process_variance', 0.01) # process variance of the EKF
        self.ekf_measurement_variance = rospy.get_param('~ekf_measurement_variance', 0.1) # measurement variance of the EKF
        
        # State variables
        self.last_scan = None
        self.last_scan_time = None
        
        # Publishers
        self.object_pos_pub = rospy.Publisher('object_position', PointStamped, queue_size=10)
        self.object_vel_pub = rospy.Publisher('object_velocity', Odometry, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('scan', LaserScan, self.laser_scan_callback)
        
    def laser_scan_callback(self, msg):
        # Save the latest scan and time
        self.last_scan = msg
        self.last_scan_time = rospy.Time.now()
        
        # Cluster the laser scan data
        clusters = self.cluster_scan(self.last_scan)
        
        # Estimate the velocity of each cluster using an EKF
        velocities = []
        for i, cluster in enumerate(clusters):
            velocity = self.estimate_velocity(cluster)
            velocities.append(velocity)
            
        # Publish the position and velocity of each object
        for i, cluster in enumerate(clusters):
            position = self.calculate_cluster_position(cluster)
            velocity = velocities[i]
            
            pos_msg = PointStamped()
            pos_msg.header.frame_id = self.last_scan.header.frame_id
            pos_msg.header.stamp = self.last_scan_time
            pos_msg.point.x = position[0]
            pos_msg.point.y = position[1]
            pos_msg.point.z = position[2]
            self.object_pos_pub.publish(pos_msg)
            
            vel_msg = Odometry()
            vel_msg.header.frame_id = self.last_scan.header.frame_id
            vel_msg.header.stamp = self.last_scan_time
            vel_msg.pose.pose.position.x = position[0]
            vel_msg.pose.pose.position.y = position[1]
            vel_msg.pose.pose.position.z = position[2]
            vel_msg.twist.twist.linear.x = velocity[0]
            vel_msg.twist.twist.linear.y = velocity[1]
            vel_msg.twist.twist.linear.z = velocity[2]
            self.object_vel_pub.publish(vel_msg)
            
            rospy.loginfo("Object at position (%f, %f, %f) moving at velocity (%f, %f, %f)", position[0], position[1], position[2], velocity[0], velocity[1], velocity[2])
        
    def cluster_scan(self, scan):
        ranges = np.array(scan.ranges)
        # Convert polar coordinates to Cartesian coordinates
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        x = np.multiply(ranges, np.cos(angles))
        y = np.multiply(ranges, np.sin(angles))
        z = np.array(scan.header.stamp.to_sec() + np.linspace(0, scan.time_increment * len(ranges), len(ranges)))
        
        # Stack the x, y, and z arrays to create an array of shape (N, 3)
        data = np.column_stack((x, y, z))
        
        # Initialize an empty list of clusters
        clusters = []
        
        # Cluster the data
        while data.shape[0] > 0:
            cluster = [data[0,:]]
            data = np.delete(data, 0, axis=0)
            
            for i in range(data.shape[0]):
                if np.linalg.norm(cluster[-1,:] - data[i,:]) < self.cluster_distance_threshold and abs(cluster[-1,2] - data[i,2]) < self.cluster_max_time_diff:
                    cluster.append(data[i,:])
                    data = np.delete(data, i, axis=0)
                    break
            
            if len(cluster) >= self.cluster_size_threshold:
                clusters.append(np.array(cluster))
        
        # # Estimate the velocity of each cluster using an EKF
        # velocities = []
        # for i, cluster in enumerate(clusters):
        #     velocity = self.estimate_velocity(cluster)
        #     velocities.append(velocity)
            
        return clusters
    
    def calculate_cluster_position(self, cluster):
        # Calculate the centroid of the cluster
        centroid = np.mean(cluster, axis=0)
        
        return centroid
    
    def estimate_velocity(self, cluster):
        # Initialize the state vector
        state = np.array([cluster[0,0], cluster[0,1], cluster[0,2], 0, 0, 0])
        
        # Initialize the state covariance matrix
        cov = np.zeros((6,6))
        cov[0,0] = self.ekf_process_variance
        cov[1,1] = self.ekf_process_variance
        cov[2,2] = self.ekf_process_variance
        cov[3,3] = self.ekf_process_variance
        cov[4,4] = self.ekf_process_variance
        cov[5,5] = self.ekf_process_variance
        
        # Initialize the measurement matrix and covariance matrix
        H = np.zeros((3,6))
        H[0,0] = 1
        H[1,1] = 1
        H[2,2] = 1
        
        R = np.zeros((3,3))
        R[0,0] = self.ekf_measurement_variance
        R[1,1] = self.ekf_measurement_variance
        R[2,2] = self.ekf_measurement_variance
        
        # Iterate over the cluster and update the EKF
        for i in range(1, cluster.shape[0]):
            # Propagate the state
            dt = cluster[i,2] - cluster[i-1,2]
            F = np.eye(6)
            F[0,3] = dt
            F[1,4] = dt
            F[2,5] = dt
            
            Q = np.zeros((6,6))
            Q[0,0] = 0.5 * dt**2 * self.ekf_process_variance
            Q[1,1] = 0.5 * dt**2 * self.ekf_process_variance
            Q[2,2] = 0.5 * dt**2 * self.ekf_process_variance
            Q[3,3] = dt * self.ekf_process_variance
            Q[4,4] = dt * self.ekf_process_variance
            Q[5,5] = dt * self.ekf_process_variance
            
            state = F.dot(state)
            cov = F.dot(cov).dot(F.T) + Q
            
            # Update the state using the measurement
            z = np.array([cluster[i,0], cluster[i,1], cluster[i,2]])
            y = z - H.dot(state)
            S = H.dot(cov).dot(H.T) + R
            K = cov.dot(H.T).dot(np.linalg.inv(S))
            state = state + K.dot(y)
            cov = (np.eye(6) - K.dot(H)).dot(cov)
        
        # Calculate the velocity from the final state estimate
        velocity = np.array([state[3], state[4], state[5]])
        
        return velocity
