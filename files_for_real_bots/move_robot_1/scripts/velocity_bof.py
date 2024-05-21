#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
from bayesian_filter import BayesianFilter

class ClusterDetector:
    def __init__(self, cluster_distance_threshold, cluster_size_threshold):
        self.cluster_distance_threshold = cluster_distance_threshold
        self.cluster_size_threshold = cluster_size_threshold
        
    def process_scan(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Convert to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Find clusters using distance thresholding
        cluster_indices = []
        visited_indices = set()
        for i in range(len(ranges)):
            if i in visited_indices:
                continue
            cluster_indices.append([i])
            visited_indices.add(i)
            for j in range(i+1, len(ranges)):
                if j in visited_indices:
                    continue
                if np.linalg.norm([x[i] - x[j], y[i] - y[j]]) < self.cluster_distance_threshold:
                    cluster_indices[-1].append(j)
                    visited_indices.add(j)

        # Filter out clusters with too few points
        clusters = []
        for indices in cluster_indices:
            if len(indices) >= self.cluster_size_threshold:
                x_mean = np.mean(x[indices])
                y_mean = np.mean(y[indices])
                clusters.append((x_mean, y_mean))

        return clusters

class ObjectTracker:
    def __init__(self, object_position_variance, object_velocity_variance):
        self.object_position_variance = object_position_variance
        self.object_velocity_variance = object_velocity_variance
        self.time_prev = None
        self.object_velocity = None
        self.filter = BayesianFilter(np.array([0, 0, 0]), np.eye(3) * object_velocity_variance, np.eye(3) * object_position_variance, 0.1)

    def process_clusters(self, clusters, time_now):
        if self.time_prev is None:
            self.time_prev = time_now
            return None
        dt = (time_now - self.time_prev).to_sec()
        if dt <= 0:
            return None
        if len(clusters) == 0:
            self.filter.predict(dt)
        else:
            measurements = np.array([np.append(c, 0) for c in clusters])
            self.filter.update(measurements)
        self.filter.predict(dt)
        self.object_velocity = self.filter.estimate_velocity()[0:2]
        self.time_prev = time_now
        return self.object_velocity

def main():
    rospy.init_node('laser_cluster_tracking')

    # Parameters
    cluster_distance_threshold = rospy.get_param('~cluster_distance_threshold', 0.2)
    cluster_size_threshold = rospy.get_param('~cluster_size_threshold', 10)
    object_position_variance = rospy.get_param('~object_position_variance', 0.01)
    object_velocity_variance = rospy.get_param('~object_velocity_variance', 1.0)

    # Publishers
    object_velocity_pub = rospy.Publisher('~object_velocity', Pose, queue_size=10)

    # Subscribers
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Objects
    cluster_detector = ClusterDetector(cluster_distance_threshold, cluster_size_threshold)
    object_tracker = ObjectTracker(object_position_variance, object_velocity_variance)

    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        # Process laser scan
        clusters = cluster_detector.process_scan(scan_msg)

        # Process clusters and estimate object velocity
        object_velocity = object_tracker.process_clusters(clusters, scan_msg.header.stamp)

        # Publish object velocity
        if object_velocity is not None:
            object_velocity_msg = Pose()
            object_velocity_msg.position.x = object_velocity[0]
            object_velocity_msg.position.y = object_velocity[1]
            object_velocity_msg.position.z = 0
            object_velocity_pub.publish(object_velocity_msg)

        rate.sleep()

if __name__ == '__main__':
    main()
