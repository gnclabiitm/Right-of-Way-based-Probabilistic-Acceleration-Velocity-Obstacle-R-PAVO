#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion
from filterpy.kalman import ExtendedKalmanFilter, jacobian


class ClusterDetector:
    def __init__(self, name):
        self.scan_sub = rospy.Subscriber(f'{name}scan', LaserScan, self.scan_callback)
        self.vel_pub = rospy.Publisher(f'{name}detected_object_velocity', Vector3, queue_size=1)
        self.scan = None
        self.clusters = []
        self.min_points_per_cluster = rospy.get_param("~min_points_per_cluster", 3)
        self.max_cluster_distance = rospy.get_param("~max_cluster_distance", 0.2)

    def scan_callback(self, scan):
        self.scan = scan

    def detect_clusters(self):
        if self.scan is None:
            return []

        ranges = np.array(self.scan.ranges)

        # Replace invalid range values with max range
        ranges[np.isnan(ranges)] = self.scan.range_max

        # Find indices of points with valid range
        valid_ranges = np.where((ranges > self.scan.range_min) & (ranges < self.scan.range_max))[0]

        # Initialize cluster list
        clusters = []

        # Iterate over all valid points
        for i in valid_ranges:
            # Check if point belongs to an existing cluster
            belongs_to_cluster = False
            for cluster in clusters:
                if self.distance(self.scan, i, cluster[-1]) < self.max_cluster_distance:
                    cluster.append(i)
                    belongs_to_cluster = True
                    break

            # Create new cluster if point does not belong to any existing cluster
            if not belongs_to_cluster:
                clusters.append([i])

        # Filter out clusters with fewer than min_points_per_cluster points
        clusters = [c for c in clusters if len(c) >= self.min_points_per_cluster]

        return clusters

    def distance(self, scan, i, j):
        angle = abs(i - j) * scan.angle_increment
        return math.sqrt(scan.ranges[i]**2 + scan.ranges[j]**2 - 2*scan.ranges[i]*scan.ranges[j]*math.cos(angle))

def state_transition_function(x, dt):
    """ State transition function for the EKF """
    # Constant velocity model
    F = np.array([[1, dt], [0, 1]])
    return np.dot(F, x)

def measurement_function(x):
    """ Measurement function for the EKF """
    # Measurement is the velocity of the object
    return x[1]

def covariance_function(x, dt, q):
    """ Covariance function for the EKF """
    # Constant velocity model with process noise
    F = np.array([[1, dt], [0, 1]])
    Q = np.array([[0.5*dt**2*q, dt*q], [dt*q, q]])
    return np.dot(F, np.dot(x, F.T)) + Q

class ObjectTracker:
    def __init__(self, name):
        self.cluster_detector = ClusterDetector(name)
        # Create EKF object
        self.filter = ExtendedKalmanFilter(dim_x=4, dim_z=2)
        self.filter.x = np.zeros((4, 1))
        self.filter.P = np.eye(4)
        self.filter.R = np.eye(2)
        self.filter.Q = np.eye(4) * 0.1

        # Set measurement function
        self.filter.H = np.array([[1, 0, 0, 0],
                                  [0, 1, 0, 0]])

        # Set measurement noise covariance
        self.filter.R *= 0.5

        # Set initial time
        self.last_update_time = rospy.Time.now().to_sec()

    def predict(self, dt):
        # Update state transition matrix
        self.filter.F = np.array([[1, 0, dt, 0],
                                  [0, 1, 0, dt],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])

        # Predict state and covariance
        self.filter.predict()

    def update(self, z):
        # Compute predicted measurement
        Hx = np.dot(self.filter.H, self.filter.x)

        # Compute innovation
        y = z - Hx

        # Compute innovation covariance
        S = np.dot(np.dot(self.filter.H, self.filter.P), self.filter.H.T) + self.filter.R

        # Compute Kalman gain
        K = np.dot(np.dot(self.filter.P, self.filter.H.T), np.linalg.inv(S))

        # Update state and covariance with measurement
        self.filter.x = self.filter.x + np.dot(K, y)
        self.filter.P = np.dot((np.eye(self.filter.x.shape[0]) - np.dot(K, self.filter.H)), self.filter.P)

    def get_state(self):
        return self.filter.x
        
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Detect object clusters
            self.cluster_detector.clusters = self.cluster_detector.detect_clusters()

            # Update object velocity estimate
            if len(self.cluster_detector.clusters) > 0:
                # Assume the first cluster is the object of interest
                cluster = self.cluster_detector.clusters[0]

                # Compute range and bearing to cluster centroid
                range_ = np.mean([self.cluster_detector.scan.ranges[i] for i in cluster])
                bearing = (cluster[-1] + cluster[0]) / 2 * self.cluster_detector.scan.angle_increment

                # Convert range and bearing to Cartesian coordinates
                x = range_ * math.cos(bearing)
                y = range_ * math.sin(bearing)

                # Time since last update
                dt = rospy.Time.now().to_sec() - self.last_update_time

                # Predict state and covariance
                self.filter.predict(dt)

                # Update last update time
                self.last_update_time = rospy.Time.now().to_sec()

                # Update measurement and state estimate
                z = np.array([[y]])
                H = np.array([[0, 1]])
                
                self.filter.update(z, HJacobian=H, Hx = np.dot(self.filter.H, self.filter.x))
                self.filter.last_update_time = rospy.Time.now().to_sec()

                # Publish velocity estimate
                velocity = Vector3(self.filter.x[1], 0, 0)
                self.cluster_detector.vel_pub.publish(velocity)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('object_tracker')
    robot_namespace = rospy.get_namespace()

    pass_rate = rospy.Rate(30)

    while not rospy.has_param(robot_namespace+'/ogm_created'):  # untill ogm_created is set turtlebot object is not constructed
        pass_rate.sleep()

    tracker = ObjectTracker(robot_namespace)
    tracker.run()
