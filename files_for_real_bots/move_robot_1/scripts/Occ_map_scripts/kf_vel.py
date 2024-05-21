#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from filterpy.kalman import KalmanFilter
import numpy as np

class ObstacleTracker:
    def __init__(self):
        self.map = None
        self.laser = None
        self.velocities = {}
        self.prev_positions = {}
        self.current_positions = {}
        self.kf_list = []
        self.dt = 0.1
        self.max_distance = 10.0 # maximum distance between current and previous position to be considered the same obstacle
        
        # Initialize Kalman filters for each obstacle
        self.num_obstacles = 10 # number of obstacles to track
        for i in range(self.num_obstacles):
            kf = KalmanFilter(dim_x=4, dim_z=2)
            kf.x = np.array([0, 0, 0, 0]) # initial state: x, y, vx, vy
            kf.P = np.eye(4) * 100 # initial covariance matrix
            kf.F = np.array([[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]]) # state transition matrix
            kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]]) # observation matrix
            kf.R = np.array([[0.01, 0], [0, 0.01]]) # measurement noise covariance matrix
            self.kf_list.append(kf)

        # Subscribe to map and laser scan topics
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

    def map_callback(self, msg):
        self.map = msg

    def laser_callback(self, msg):
        self.laser = msg

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if self.map is not None and self.laser is not None:
                # Transform robot's position to map's frame of reference
                (trans, rot) = self.tf_listener.lookupTransform(self.map.header.frame_id, self.laser.header.frame_id, rospy.Time(0))
                robot_x, robot_y, robot_yaw = self.get_robot_pose(trans, rot)

                # Iterate over all cells in the map and detect obstacles
                for i in range(self.map.info.width):
                    for j in range(self.map.info.height):
                        index = j * self.map.info.width + i
                        if self.map.data[index] > 50: # obstacle detected
                            # Convert cell coordinates to world coordinates
                            x = i * self.map.info.resolution + self.map.info.origin.position.x
                            y = j * self.map.info.resolution + self.map.info.origin.position.y
                            
                            # Find the closest obstacle in the previous map update
                            min_distance = float("inf")
                            min_kf_index = None
                            for k in range(self.num_obstacles):
                                if k in self.prev_positions:
                                    distance = np.sqrt((x - self.prev_positions[k][0])**2 + (y - self.prev_positions[k][1])**2)
                                    if distance < min_distance:
                                        min_distance = distance
                                        min_kf_index = k
                                        
                           
# import rospy
# from nav_msgs.msg import OccupancyGrid
# from filterpy.kalman import KalmanFilter
# import numpy as np

# class ObstacleTracker:
#     def __init__(self):
#         self.map = None
#         self.velocities = {}
#         self.prev_positions = {}
#         self.current_positions = {}
#         self.kf_list = []
#         self.dt = 0.1
#         self.max_distance = 10.0 # maximum distance between current and previous position to be considered the same obstacle
        
#         # Initialize Kalman filters for each obstacle
#         self.num_obstacles = 10 # number of obstacles to track
#         for i in range(self.num_obstacles):
#             kf = KalmanFilter(dim_x=4, dim_z=2)
#             kf.x = np.array([0, 0, 0, 0]) # initial state: x, y, vx, vy
#             kf.P = np.eye(4) * 100 # initial covariance matrix
#             kf.F = np.array([[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]]) # state transition matrix
#             kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]]) # observation matrix
#             kf.R = np.array([[0.01, 0], [0, 0.01]]) # measurement noise covariance matrix
#             self.kf_list.append(kf)

#         # Subscribe to map topic
#         rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

#     def map_callback(self, msg):
#         self.map = msg

#     def run(self):
#         rate = rospy.Rate(10) # 10 Hz
#         while not rospy.is_shutdown():
#             if self.map is not None:
#                 # Iterate over all cells in the map and detect obstacles
#                 for i in range(self.map.info.width):
#                     for j in range(self.map.info.height):
#                         index = j * self.map.info.width + i
#                         if self.map.data[index] > 50: # obstacle detected
#                             # Convert cell coordinates to world coordinates
#                             x = i * self.map.info.resolution + self.map.info.origin.position.x
#                             y = j * self.map.info.resolution + self.map.info.origin.position.y
                            
#                             # Find the closest obstacle in the previous map update
#                             min_distance = float("inf")
#                             min_kf_index = None
#                             for k in range(self.num_obstacles):
#                                 if k in self.prev_positions:
#                                     distance = np.sqrt((x - self.prev_positions[k][0])**2 + (y - self.prev_positions[k][1])**2)
#                                     if distance < min_distance:
#                                         min_distance = distance
#                                         min_kf_index = k
                                        
#                             if min_kf_index is not None and min_distance < self.max_distance:
#                                 # Update Kalman filter with new observation
#                                 kf = self.kf_list[min_kf_index]
#                                 measurement = np.array([[x], [y]])
#                                 kf.update(measurement)
                                
#                                 # Store current and previous positions of obstacle
#                                 self.current_positions[min_kf_index] = (x, y)
#                                 self.prev_positions.pop(min_kf_index, None)
#                             else:
#                                 # Create a new Kalman filter for the obstacle
#                                 kf_index = None
#                                 for k in range(self.num_obstacles):
#                                     if

#!/usr/bin/env python

# import rospy
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import TwistWithCovarianceStamped
# from filterpy.kalman import KalmanFilter
# from numpy import dot, eye, array

# class ObstacleVelocityEstimator:
#     def __init__(self):
#         # Initialize ROS node
#         rospy.init_node('obstacle_velocity_estimator', anonymous=True)

#         # Initialize subscribers and publishers
#         self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
#         self.velocity_pub = rospy.Publisher('/obstacle_velocity', TwistWithCovarianceStamped, queue_size=10)

#         # Initialize class variables
#         self.grid_map = None
#         self.grid_resolution = None
#         self.kf = None

#     def map_callback(self, map):
#         # Save map data and resolution
#         self.grid_map = array(map.data).reshape(map.info.height, map.info.width)
#         self.grid_resolution = map.info.resolution

#         # Initialize Kalman filter if not already initialized
#         if self.kf is None:
#             self.kf = KalmanFilter(dim_x=4, dim_z=2)
#             self.kf.x = array([0.0, 0.0, 0.0, 0.0]).T
#             self.kf.P = eye(4) * 1000
#             self.kf.R = eye(2) * 10
#             self.kf.Q = eye(4) * 0.01

#         # Predict and update Kalman filter using map data
#         if self.kf is not None:
#             dt = 1.0 / rospy.get_param('~rate')
#             self.kf.F = array([[1, 0, dt, 0],
#                                [0, 1, 0, dt],
#                                [0, 0, 1, 0],
#                                [0, 0, 0, 1]])
#             self.kf.predict()
#             for i in range(self.grid_map.shape[0]):
#                 for j in range(self.grid_map.shape[1]):
#                     if self.grid_map[i, j] > 0:
#                         x = j * self.grid_resolution
#                         y = i * self.grid_resolution
#                         z = array([x, y])
#                         H = array([[1, 0, 0, 0],
#                                    [0, 1, 0, 0]])
#                         self.kf.update(z, H)

#             # Publish estimated velocity as TwistWithCovarianceStamped message
#             velocity_msg = TwistWithCovarianceStamped()
#             velocity_msg.header.stamp = rospy.Time.now()
#             velocity_msg.twist.twist.linear.x = self.kf.x[2]
#             velocity_msg.twist.twist.linear.y = self.kf.x[3]
#             velocity_msg.twist.covariance = self.kf.P.flatten().tolist()
#             self.velocity_pub.publish(velocity_msg)

# if __name__ == '__main__':
#     try:
#         estimator = ObstacleVelocityEstimator()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from filterpy.kalman import KalmanFilter

NUM_OBS = 3 # number of obstacles

class KalmanVelEstimator:

    def __init__(self):
        self.obstacle_poses = np.zeros((NUM_OBS,3)) # obstacle poses [x,y,theta]
        self.obstacle_vels = np.zeros((NUM_OBS,2)) # obstacle velocities [vx,vy]
        self.filter_list = [] # list of Kalman filters for each obstacle

        # Initialize Kalman filters
        for i in range(NUM_OBS):
            kf = KalmanFilter(dim_x=4, dim_z=2)
            kf.F = np.array([[1.0, 0.0, 0.1, 0.0],
                             [0.0, 1.0, 0.0, 0.1],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])
            kf.H = np.array([[0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])
            kf.R = np.array([[0.01, 0.0],
                             [0.0, 0.01]])
            kf.P = np.diag([1.0, 1.0, 1.0, 1.0])
            self.filter_list.append(kf)

        # Subscribe to laser data and occupancy grid map
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.grid_sub = rospy.Subscriber('/map', OccupancyGrid, self.grid_callback)

        # Publish estimated obstacle poses and Turtlebot3 velocity
        self.obstacle_poses_pub = rospy.Publisher('/obstacle_poses', PoseArray, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def laser_callback(self, scan_msg):
        # Extract laser data and estimate obstacle velocities using Kalman filters
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        for i in range(NUM_OBS):
            # TODO: implement obstacle detection and pose estimation from laser data
            # For now, assume that obstacle poses are known
            vel_x = 0.0
            vel_y = 0.0
            if i == 0:
                vel_x = 0.1
                vel_y = 0.0
            elif i == 1:
                vel_x = 0.0
                vel_y = 0.1
            elif i == 2:
                vel_x = -0.1
                vel_y = -0.1

            # Update Kalman filter
            kf = self.filter_list[i]
            kf.predict()
            kf.update(np.array([vel_x, vel_y]))
            self.obstacle_vels[i,:] = kf.x
            
    def grid_callback(self, grid_msg):
        # Extract occupancy grid map and update obstacle poses using estimated velocities
        # For now, assume that obstacle poses are updated with constant velocity model
        resolution = grid_msg.info.resolution
        origin_x = grid_msg.info.origin.position.x
        origin_y = grid_msg.info.origin.position.y
        data = np.array(grid_msg.data).reshape(grid_msg.info.height, grid_msg.info.width)
        for i in range(NUM_OBS):
            pose = self.obstacle_poses[i,:]
            vel = self.obstacle_vels[i,:]
            pose[0] += vel[0] * np.cos(pose[2]) * resolution
            pose[1] += vel[1] * np.sin(pose[2]) * resolution
            pose[2] = np.arctan2(vel[1], vel[0])
            pose[0] = np.clip(pose[0], 0.0, data.shape[1] * resolution)
            pose[1] = np.clip(pose[1], 0.0, data.shape[0] * resolution)
            self.obstacle_poses[i,:] = pose

        # Publish estimated obstacle poses
        poses_msg = PoseArray()
        poses_msg.header.stamp = rospy.Time.now()
        poses_msg.header.frame_id = 'map'
        for i in range(NUM_OBS):
            pose = self.obstacle_poses[i,:]
            p = PoseStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = 'map'
            p.pose.position.x = pose[0] + origin_x
            p.pose.position.y = pose[1] + origin_y
            p.pose.position.z = 0.0
            p.pose.orientation.z = np.sin(pose[2]/2)
            p.pose.orientation.w = np.cos(pose[2]/2)
            poses_msg.poses.append(p)
        self.obstacle_poses_pub.publish(poses_msg)

        # Publish Turtlebot3 velocity
        cmd_vel_msg = Twist()
        # TODO: implement control law to follow detected obstacles
        cmd_vel_msg.linear.x = 0.1
        self.cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == '__main__':
    rospy.init_node('kalman_vel_estimator', anonymous=True)
    estimator = KalmanVelEstimator()
    rospy.spin()
