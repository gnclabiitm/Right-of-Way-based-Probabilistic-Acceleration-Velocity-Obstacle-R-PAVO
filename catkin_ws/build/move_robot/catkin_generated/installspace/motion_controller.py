#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from obstacle_detector.msg import Obstacles
from grid_map_msgs.msg import GridMap
from tf.transformations import euler_from_quaternion
from std_msgs.msg import ColorRGBA, Float32MultiArray, Bool
from move_robot.msg import plot_data, mission_plot_data
from visualization_msgs.msg import Marker
from move_robot.srv import SyncMotion, SyncMotionResponse
from rospy import Duration
import math
import numpy as np
import rosbag

import sys

SCRIPTS_PATH = '/home/vivek/catkin_ws/src/move_robot/scripts'
sys.path.insert(0, SCRIPTS_PATH)

from obstacles_utils import *

class Agent:

    def __init__(self, name, goal_x, goal_y, map_x_limit, map_y_limit, map_resolution):
        # Initialize the node and subscribers/publishers
        self.name = name
        # rospy.init_node('controller')
        # rospy.resolve_name(f'{name}_controller', caller_id=None)
        self.plot_pub = rospy.Publisher(f'{name}plot_data', plot_data, queue_size=10)
        self.velocity_publisher = rospy.Publisher(f'{name}cmd_vel', Twist, queue_size=1)
        self.points_pub = rospy.Publisher('next_reachable_velocity_points', Marker, queue_size=1)
        self.goal_stat_pub = rospy.Publisher(f'{name}goal_reached', Bool, queue_size=10)
        self.mission_plot_data_pub = rospy.Publisher(f'{name}mission_plot_data', mission_plot_data, queue_size=10)
        self.odom_subscriber = rospy.Subscriber(f'{name}odom', Odometry, self.odom_callback)
        self.obstacle_subsciber = rospy.Subscriber(f'{name}obstacles', Obstacles, self.circle_obstacles_callback)
        self.ogm_subscriber = rospy.Subscriber(f'{name}ogm', GridMap, self.ogm_callback)
        self.goal_reached =  False
        self.map_x_len = map_x_limit
        self.map_y_len = map_y_limit
        self.map_res = map_resolution
        self.obst_tags = []
        self.ogm =[[],[],[],[],[]]
        self.obs_occ = []
        self.obs_occ_vel_x = []
        self.obs_occ_vel_y = []
        self.obs_occ_pos_x = []
        self.obs_occ_pos_y = []
        self.obstacles = []
        self.obst_center = []
        self.obst_vel = []
        self.ROW = None
        self.AVO = None
        self.stat_obst = None
        self.proc_var = 0.01
        self.proc_rate_var = 0.1
        self.meas_var = 1
        self.proc_rate = 2
        self.vel_variance = self.proc_var + self.proc_rate*self.proc_rate_var + self.meas_var
        self.rate = rospy.Rate(2) # 2hz

        # Initialize the goal and current position of the turtlebot
        self.goal = Point(goal_x, goal_y, 0)
        self.current_odom = None
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.current_speed = 0.0
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0
        self.desired_vel_x = 0.0
        self.desired_vel_y = 0.0
        self.kp = 1.0 
        self.goal_tol = 0.25
        #Set the maximum velocities (from turtlebot3 data)
        self.max_linear_vel = 0.22
        self.max_angular_vel = 2.8
        self.tH = 2.0
        self.delta = 1.0
        self.t_iter = 0.5
        # self.a_max = (self.max_linear_vel*2)/self.delta
        self.a_max = 0.22
        self.safe_rad = 0.25 # 6*(self.delta**2)*self.a_max     #6*delta*V_max - Radius_agent   #usually I take 0.3

        #Set the initial velocities
        self.linear_vel = 0.05
        self.angular_vel = 0.1
        self.acceleration = []
        self.avoid = 0.0
        self.a_long =0.0
        self.a_lat =0.0
        self.prev_linear_vel = 0.0
        self.prev_time = 0.0
        self.a_long_lv = 0.0
        self.a_lat_wv = 0.0
        self.acc = 0.0

        # Set the initials for bot stuck check
        # self.odom_data_history = []
        # self.stuck_threshold = 0.002  # Change this threshold as needed
        # self.history_size = 150  # Number of odom data samples to store


        # #Set the initials for plotting data
        self.time_data = []
        self.velocity_x_data = []
        self.velocity_y_data = []
        self.velocity_mag_data = []
        self.accel_long_data = []
        self.accel_lat_data = []
        self.accel_mag_data = []
        self.avoidance_data = []

        #Set the initials for box plotting data
        self.goal_stat = False
        self.d_mini = []
        self.d_min = 1000.0
        self.t_compu = []
        self.t_comp = 0.0
        self.t_mission = 0.0
        self.accel = []
        self.acc_rms = 0.0

    def update_plotting_data(self, current_vel_x, current_vel_y, desired_vel_x, desired_vel_y):
         # Velocity and Acceleration computing for plotting graphs w.r.t. time.
            a_x = (desired_vel_x - current_vel_x)/self.delta
            a_y = (desired_vel_y - current_vel_y)/self.delta
            v = [current_vel_x, current_vel_y]
            a = [a_x, a_y]
            ang = angle_finder(v,a)
            a_mag = np.linalg.norm(a)
            a_long = -a_mag*math.cos(ang)
            a_lat = a_mag*math.sin(ang)
            a_net = np.linalg.norm([a_long, a_lat])
            # a_net = math.sqrt(a_long^2 + a_lat^2)
            self.acceleration = [a_long, a_lat]
            self.a_long = a_long
            self.a_lat = a_lat

            current_time_0 = rospy.Time.now()
            dt = current_time_0.to_sec() - self.prev_time
            self.prev_time = current_time_0.to_sec()
            self.a_long_lv = (self.linear_vel - self.prev_linear_vel)/dt
            self.a_lat_wv = (self.linear_vel)*(self.angular_vel)
            self.prev_linear_vel = self.linear_vel
            a_net_2 = np.linalg.norm([self.a_long_lv, self.a_lat_wv])
            self.acc = math.sqrt((self.a_long_lv)**2 + (self.a_lat_wv)**2)

            current_time_1 = rospy.Time.now()
            self.time_data.append(current_time_1.to_sec())
            self.velocity_x_data.append(current_vel_x)
            self.velocity_y_data.append(current_vel_y)
            self.velocity_mag_data.append(np.linalg.norm(v))
            self.accel_long_data.append(self.a_long_lv)
            self.accel_lat_data.append(self.a_lat_wv)
            self.accel_mag_data.append(a_net_2)
            self.avoidance_data.append(self.avoid)
            self.accel.append((self.a_long_lv)**2 + (self.a_lat_wv)**2)


    def update_velocity(self, t_it, lin_speed, orient, desired_vel_x, desired_vel_y):
        # self.current_vel_x = lin_speed*math.cos(orient)
        # self.current_vel_y = lin_speed*math.sin(orient)
        # while True:
            # Code for the small while loop
            # print("Small While Loop: Doing some work")
            
        t_iter = t_it
        self.current_vel_x = desired_vel_x - math.exp(-t_iter/self.delta)*(desired_vel_x-self.current_vel_x)
        self.current_vel_y = desired_vel_y - math.exp(-t_iter/self.delta)*(desired_vel_y-self.current_vel_y)
        self.update_plotting_data(self.current_vel_x, self.current_vel_y, desired_vel_x, desired_vel_y)
            # time.sleep(0.1)


    def odom_callback(self, data):
        # Update the current position of the turtlebot
        self.current_odom = data
        self.current_odom_x = self.current_odom.pose.pose.position.x
        self.current_odom_y = self.current_odom.pose.pose.position.y
        self.ornt_q = data.pose.pose.orientation
        ornt_list = [self.ornt_q.x, self.ornt_q.y, self.ornt_q.z, self.ornt_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(ornt_list)
        #rospy.loginfo(current_odom.pose.pose.position.x)

        # if len(self.odom_data_history) >= self.history_size:
        #     self.odom_data_history.pop(0)  # Remove the oldest data
        # self.odom_data_history.append(data)

    # def check_if_stuck(self):
    #     if len(self.odom_data_history) >= self.history_size:
    #         for i in range(1, len(self.odom_data_history)):
    #             current_pose = self.odom_data_history[i].pose.pose.position
    #             prev_pose = self.odom_data_history[i - 1].pose.pose.position
    #             distance = math.sqrt((current_pose.x - prev_pose.x)**2 + (current_pose.y - prev_pose.y)**2)
    #             if distance >= self.stuck_threshold:
    #                 return False
    #         return True
    #     return False

    def circle_obstacles_callback(self, data):
        self.num_obst = len(data.circles)
        self.obstacles = []
        self.obst_center = []
        self.obst_vel = []
        for i in range(self.num_obst):
            self.obstacles.append(data.circles[i])
            self.obst_center.append(data.circles[i].center)
            self.obst_vel.append(data.circles[i].velocity)

    def ogm_callback(self,data):
        row_len = int(np.sqrt(len(data.data[0].data)))
        col_len = int(np.sqrt(len(data.data[0].data)))
        self.obs_occ = np.asarray(data.data[0].data).reshape(row_len,col_len)
        self.obs_occ_vel_x = np.asarray(data.data[1].data).reshape(row_len,col_len)
        self.obs_occ_vel_y = np.asarray(data.data[2].data).reshape(row_len,col_len)
        self.obs_occ_pos_x = np.asarray(data.data[3].data).reshape(row_len,col_len)
        self.obs_occ_pos_y = np.asarray(data.data[4].data).reshape(row_len,col_len)
        self.ogm = [self.obs_occ, self.obs_occ_vel_x, self.obs_occ_vel_y, self.obs_occ_pos_x, self.obs_occ_pos_y]

    def RVS_publisher(self,rvs_pub):

        # Define the Marker message for the next reachable velocity points
        points_marker = Marker()
        # points_marker.header.frame_id = "odom" # Change the frame_id if necessary
        points_marker.header.frame_id = f"{self.name[1:]}odom" # Change the frame_id if necessary
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = 0.01 # Change the size of the points if necessary
        points_marker.scale.y = 0.01
        points_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0) # Change the color of the points if necessary

        # Define the array of x and y velocity points
        x_points = rvs_pub[0]+self.current_odom_x # Replace with your actual x velocity points
        y_points = rvs_pub[1]+self.current_odom_y # Replace with your actual y velocity points

        # Define an empty array for the next reachable velocity points
        points = []

        # Loop through the x and y points and create a new Point message for each point
        for i in range(len(x_points)):
            point = Point()
            point.x = x_points[i]
            point.y = y_points[i]
            point.z = 0.0 # Set the z-coordinate to 0.0 (assuming a 2D space)
            points.append(point)

        # print("length of points", str(len(points)))
        # print("rvs_x",rvs_pub[0])
        # Loop through the next reachable velocity points and add them to the Marker message
        for point in points:
            points_marker.points.append(point)

        self.points_pub.publish(points_marker)


    def move_to_goal(self):
        
        # rospy.loginfo_throttle(5,"Agent"+self.name+"is moving towards its goal \n")
        pos_x = self.current_odom_x
        pos_y = self.current_odom_y
        self.t_iter = 0.5#10*self.delta
        # print("current pose", pos_x, pos_y)
        # print("goal points", goal_x, goal_y)
        
        distance, angle, theta = find_los_angle(self.goal.x, self.goal.y, pos_x, pos_y,self.yaw)

        # self.desired_vel_x_goal = self.max_linear_vel*math.cos(theta)
        # self.desired_vel_y_goal = self.max_linear_vel*math.sin(theta)
        # self.update_velocity(self.t_iter,self.linear_vel, self.yaw, self.desired_vel_x_goal, self.desired_vel_y_goal)
        
        if (angle<-0.1 or angle>0.1):
            # print("correct course towards goal")
            self.angular_vel = self.kp*angle
            self.linear_vel = 0.22
            # self.linear_vel = np.sqrt((self.current_vel_x)**2 + (self.current_vel_y)**2)
            # vel_ang_des = math.atan2(self.current_vel_y,self.current_vel_x)
            # self.angular_vel = ((vel_ang_des-self.yaw)/self.delta)
        else:
            # print("simply move in the same direction")
            self.angular_vel = 0.0
            self.linear_vel = 0.22

        self.desired_vel_x_goal = self.linear_vel*math.cos(theta)
        self.desired_vel_y_goal = self.linear_vel*math.sin(theta)
        self.avoid = 0
        self.update_velocity(self.t_iter,self.linear_vel, self.yaw, self.desired_vel_x_goal, self.desired_vel_y_goal)

        # self.linear_vel = np.sqrt((self.current_vel_x)**2 + (self.current_vel_y)**2)
        # vel_ang_des = math.atan2(self.current_vel_y,self.current_vel_x)
        # self.angular_vel = ((vel_ang_des-self.yaw)/self.delta)

        #Check if the goal is reached (within the tolerance range)
        #come out of the loop and send zero velocities
        if distance < self.goal_tol:
            self.goal_reached = True
             

    #Function to move the turtlebot to its goal position
    def move_turtlebot_to_position(self, t_mission_0_temp):      
        command_num = 0
        t_mission_check_1 = 0.0
        prev_odom_x = 0.0
        #Loop to run the robot towards the goal untill it reaches or if terminal shutdown is done
        while not rospy.is_shutdown():
            # small_loop_thread = threading.Thread(target=self.update_velocity)
            # small_loop_thread.start()
            #Calculating the distance and angle between the robot's current and goal position
            if self.current_odom is not None:
                t_comp_0 = rospy.Time.now()

                self.obst_tags, d_m = obstacles_under_horizon(self.current_odom, self.obstacles)
                if d_m > 10000: 
                    d_m = 1
                self.d_mini.append(d_m)
                # print(len(self.obst_center))
                # print("obstacles",self.obstacles)
                # print("obstacle Tag",self.obst_tags)

                # if len(self.obst_tags) < 1: #if there are no obstacles, then  move towards goal
                if np.sum(self.obs_occ) == 0.0:                    
                    # rospy.loginfo("Agent "+self.name+" has found NO Obstacles")
                    self.move_to_goal()
                    # if not self.goal_reached:
                    #     current_time = rospy.Time.now().to_sec()
                    #     time_since_start = current_time - t_mission_0_temp.to_sec()
                    #     if time_since_start > 100:
                    #         break
                    if self.goal_reached:
                        # self.goal_stat = True
                        # rospy.loginfo("Agent "+self.name+" has reached its Goal position")
                        break

                else: #if obstacles found within the sensing range
                    # rospy.loginfo("Agent "+self.name+" has found "+str(len(self.obst_tags))+" Obstacles")
                    # rospy.loginfo("Agent "+self.name+" has found Obstacles")
                    # print("obstacle data", self.obst_tags)
                    # self.ROW, self.stat_obst= Encounter_type(self.obst_tags, self.yaw, self.safe_rad, self.current_odom, self.current_vel_x, self.current_vel_y, self.map_x_len, self.map_y_len)
                    self.ROW, self.stat_obst= Encounter_type_2(self.obst_tags, self.yaw, self.safe_rad, self.current_odom, self.current_vel_x, self.current_vel_y, self.map_x_len, self.map_y_len)
                    # print("ROW of",self.name,"is:", self.ROW)
                    if (self.ROW and self.stat_obst): #if agent has right-of-way w.r.t all the other agents
                        # rospy.loginfo("Agent "+self.name+" has ROW")
                        self.move_to_goal()
                        # if not self.goal_reached:
                        #     current_time = rospy.Time.now().to_sec()
                        #     time_since_start = current_time - t_mission_0_temp.to_sec()
                        #     if time_since_start > 100:
                        #         break
                        if self.goal_reached:
                            # self.goal_stat = True
                            # rospy.loginfo("Agent "+self.name+" has reached its Goal position")
                            break

                    else:
                        # rospy.loginfo("Agent "+self.name+" has found "+str(len(self.obst_tags))+" Obstacles")
                        self.desired_vel_x, self.desired_vel_y, rvs_pub = PVO_Controller(self.ogm,self.current_odom_x,self.current_odom_y, self.current_vel_x, self.current_vel_y,self.yaw,self.tH, self.a_max, 
                                       self.delta,self.goal.x,self.goal.y, self.safe_rad,self.vel_variance)
                        # print("RVS", rvs_pub)
                        self.RVS_publisher(rvs_pub)
                        self.t_iter = 0.5
                        self.avoid = 1
                        self.update_velocity(self.t_iter,self.linear_vel, self.yaw, self.desired_vel_x, self.desired_vel_y)
                        # t_iter = 0.5
                        # self.current_vel_x = self.desired_vel_x - math.exp(-t_iter/self.delta)*(self.desired_vel_x-self.current_vel_x)
                        # self.current_vel_y = self.desired_vel_y - math.exp(-t_iter/self.delta)*(self.desired_vel_y-self.current_vel_y)
                        # self.linear_vel = np.sqrt((self.desired_vel_x)**2 + (self.desired_vel_y)**2)
                        self.linear_vel = np.sqrt((self.current_vel_x)**2 + (self.current_vel_y)**2)
                        # vel_ang_des = math.atan2(self.desired_vel_y,self.desired_vel_x)
                        vel_ang_des = math.atan2(self.current_vel_y,self.current_vel_x)
                        self.angular_vel = ((vel_ang_des-self.yaw)/self.delta)
                        
                        # self.update_plotting_data(self.current_vel_x, self.current_vel_y, self.desired_vel_x, self.desired_vel_y)
                        # rospy.loginfo("Agent "+self.name+" is avoding")
                        # print("Desired velocity of",self.name,"is:",self.desired_vel_x, self.desired_vel_y, "\n")
                        # rospy.loginfo_throttle(1,"Agent "+self.name+"'s V_x "+str(self.desired_vel_x)+" and V_y "+str(self.desired_vel_y)+ "\n")
                        # rospy.loginfo_throttle(1,"Agent "+self.name+"'s lin vel "+ str(self.linear_vel)+" and ang vel "+str(self.angular_vel)+ "\n")
                        # print("command_num",command_num)
                        command_num += 1

        
                self.t_comp = rospy.Time.now() - t_comp_0
                self.t_compu.append(self.t_comp)
           
            
            #Creating a twist message to set the velocities for the later publishment
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_vel  #since directed towards goal
            twist_msg.angular.z = self.angular_vel  #since only yaw rotation (along z) is possible

            pd = plot_data()
            pd.curr_velocity_x = self.current_vel_x
            pd.curr_velocity_y = self.current_vel_y
            pd.des_velocity_x = self.desired_vel_x
            pd.des_velocity_y = self.desired_vel_y
            pd.t_iter = self.t_iter
            pd.delta = self.delta
            pd.t_horizon = self.tH
            pd.avoid_i = self.avoid
            pd.a_long = self.a_long
            pd.a_lat = self.a_lat
            pd.vel_mag = self.linear_vel
            pd.a_long_lv = self.a_long_lv
            pd.a_lat_wv = self.a_lat_wv
            pd.accel =  self.acc
            # pd.t_comp = self.t_comp

            mpd_1 = mission_plot_data()
            mpd_1.t_mission = 0.0
            mpd_1.d_min = 0.0
            mpd_1.t_comp = 0.0
            mpd_1.a_rms = 0.0

            self.plot_pub.publish(pd)

            self.mission_plot_data_pub.publish(mpd_1)

            if not self.goal_reached:
                current_time = rospy.Time.now().to_sec()
                time_since_start = current_time - t_mission_0_temp.to_sec()
                if time_since_start > 300:
                    break

            # if self.check_if_stuck():
            #     # Bot is stuck, apply backward linear velocity
            #     twist = Twist()
            #     twist.linear.x = -0.22
            #     # self.velocity_publisher.publish(twist)

            self.goal_stat_pub.publish(self.goal_stat)

            # if self.linear_vel > 0.001:
            #     if (self.current_odom_x - prev_odom_x) <= 0.001 :
            #         self.linear_vel = -0.2
            #         twist_msg = Twist()
            #         twist_msg.linear.x = self.linear_vel

            #Publishing the twist messages to the '\cmd_vel' topic
            self.velocity_publisher.publish(twist_msg)
            # rospy.loginfo_throttle(5,"twist_msg is published \n")

            prev_odom_x = self.current_odom_x

            rospy.sleep(0.5)
        
        #stop the robot by publishing zero velocities to /cmd_vel 
        twist_msg = Twist()
        self.velocity_publisher.publish(twist_msg) #default Twist will give zero velocities

# def trigger_sync_motion_client(trigger):
    
#     rospy.wait_for_service('trigger_sync_motion')
#     try:
        
#         trigger_motion = rospy.ServiceProxy('trigger_sync_motion', SyncMotion)
#         response = trigger_motion(trigger)
#         return response.sync_success
#     except rospy.ServiceException as e:
#         rospy.logerr("Sync Service call failed: %s", e)
#         return False

# def handle_trigger_motion(req):
#     rospy.loginfo("Received motion trigger request: %s", req.trigger)

#     # Here, you can add your motion controller logic.
#     # When req.trigger is True, start motion control.

#     # Simulate motion control for testing.
#     if req.trigger:
#         # Start motion controller logic
#         rospy.loginfo("Starting motion control...")
#         return SyncMotionResponse(True)

#     return SyncMotionResponse(False)


if __name__ == '__main__':
    try:
        rospy.init_node('tb3_motion_controller')
        # Get the namespace and goal parameters from the launch file
        # robot_namespace = rospy.get_param('~robot_namespace', '/tb3_0')
        robot_namespace = rospy.get_namespace()
        # mission_plot_data_pub = rospy.Publisher(f'{robot_namespace}mission_plot_data', mission_plot_data, queue_size=10)

        pass_rate = rospy.Rate(30)

        while not rospy.has_param(robot_namespace+'/ogm_created'):  # untill ogm_created is set turtlebot object is not constructed
            pass_rate.sleep()


        if rospy.has_param(robot_namespace+'/ogm_created'):
            rospy.loginfo('ogm_created parameter has been set')
        else:
            rospy.logwarn('ogm_created parameter is not set')

        goal_x = rospy.get_param('~goal_x', 0.0)
        goal_y = rospy.get_param('~goal_y', 0.0)
        map_x_limit = rospy.get_param("/map_x_limit")
        map_y_limit = rospy.get_param("/map_y_limit")
        map_resolution = rospy.get_param("/map_resolution")
               
        while not rospy.has_param('/motion_sync_param'):  # untill ogm_created is set turtlebot object is not constructed
            pass_rate.sleep()

        t_mission_0 = rospy.Time.now()
        # Create a turtlebot object and move it to the goal
        turtlebot = Agent(robot_namespace,goal_x, goal_y, map_x_limit, map_y_limit, map_resolution)
        turtlebot.move_turtlebot_to_position(t_mission_0)

        # Maximum time taken by the agent to accomplish the mission
        turtlebot.t_mission = rospy.Time.now() - t_mission_0
        # print('t_mission', turtlebot.t_mission.to_sec(), '\n')

        # Minimum distance encounterd by agent w.r.t. any other agent throughout the mission
        turtlebot.d_min = min(turtlebot.d_mini)
        # print('d_min', turtlebot.d_min, '\n')

        # Average time taken to compute the safe velocity for the next planning time-horizon throughtout the mission
        # print(turtlebot.t_compu)
        t_comp_test = []
        t_comp_sum = Duration()
        for interval in turtlebot.t_compu:
            t_comp_sum += interval
            t_comp_test.append(interval.nsecs)
        turtlebot.t_comp = t_comp_sum/len(turtlebot.t_compu)
        # print('t_comp_test',t_comp_test,'\n')
        # print('t_comp', turtlebot.t_comp.to_nsec(),'\n')

        # RMS value of the magnitude of an agent’s acceleration over the course of the mission.
        turtlebot.acc_rms = math.sqrt(sum(turtlebot.accel))/len(turtlebot.accel)
        # print('acc_rms', turtlebot.acc_rms, '\n')

        # print('t_mission, d_min, t_comp, acc_rms', turtlebot.t_mission.to_sec(), turtlebot.d_min, turtlebot.t_comp.to_sec(), turtlebot.acc_rms)

        mpd_2 = mission_plot_data()
        mpd_2.t_mission = turtlebot.t_mission.to_sec()
        mpd_2.d_min = turtlebot.d_min
        # scaling_fac = 10**(-6)
        # mpd.t_comp = ((turtlebot.t_comp)*1000000)*scaling_fac
        mpd_2.t_comp = turtlebot.t_comp.to_nsec()
        mpd_2.a_rms = turtlebot.acc_rms

        turtlebot.mission_plot_data_pub.publish(mpd_2)

        turtlebot.goal_stat =  True
        turtlebot.goal_stat_pub.publish(turtlebot.goal_stat)


        vel_plots = True
        # if vel_plots:        
        plot_params = ['time','velocity_x','velocity_y','velocity_mag','accel_long','accel_lat','accel_mag','avoidance']

        bag_0 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[0] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_1 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[1] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_2 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[2] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_3 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[3] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_4 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[4] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_5 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[5] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_6 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[6] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_7 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[7] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')

        # bags = {bag_0, bag_1,bag_2,bag_3,bag_4,bag_5,bag_6}
        # print(turtlebot.accel_lat_data)
        time_data = Float32MultiArray()
        velocity_x_data = Float32MultiArray()
        velocity_y_data = Float32MultiArray()
        velocity_mag_data = Float32MultiArray()
        accel_long_data = Float32MultiArray()
        accel_lat_data = Float32MultiArray()
        accel_mag_data = Float32MultiArray()
        avoidance_data = Float32MultiArray()

        time_data.data = turtlebot.time_data
        velocity_x_data.data = turtlebot.velocity_y_data
        velocity_y_data.data = turtlebot.velocity_x_data
        velocity_mag_data.data = turtlebot.velocity_mag_data
        accel_lat_data.data = turtlebot.accel_lat_data
        accel_long_data.data = turtlebot.accel_long_data        
        accel_mag_data.data = turtlebot.accel_mag_data
        avoidance_data.data = turtlebot.avoidance_data

        bag_0.write(robot_namespace.replace('/', '')+'_'+plot_params[0]+'_data', time_data)
        bag_1.write(robot_namespace.replace('/', '')+'_'+plot_params[1]+'_data', velocity_x_data)
        bag_2.write(robot_namespace.replace('/', '')+'_'+plot_params[2]+'_data', velocity_y_data)
        bag_3.write(robot_namespace.replace('/', '')+'_'+plot_params[3]+'_data', velocity_mag_data)
        bag_4.write(robot_namespace.replace('/', '')+'_'+plot_params[4]+'_data', accel_long_data)
        bag_5.write(robot_namespace.replace('/', '')+'_'+plot_params[5]+'_data', accel_lat_data)
        bag_6.write(robot_namespace.replace('/', '')+'_'+plot_params[6]+'_data', accel_mag_data)
        bag_7.write(robot_namespace.replace('/', '')+'_'+plot_params[7]+'_data', avoidance_data)
        # print(bag_0)

        

        rospy.signal_shutdown('Agent reached finished its mission successfully')
    except rospy.ROSInterruptException:
        rospy.logerr("tb3 motion controller failed")
        pass
    finally:
        bag_0.close()
        bag_1.close()
        bag_2.close()
        bag_3.close()
        bag_4.close()
        bag_5.close()
        bag_6.close()
        bag_7.close()

        
