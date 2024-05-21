#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from move_robot.msg import plot_data
import math
import numpy as np
import rosbag
import sys

SCRIPTS_PATH = '/home/ubuntu/catkin_ws/src/move_robot/scripts'
sys.path.insert(0, SCRIPTS_PATH)

from obstacles_utils import angle_finder


class Recoder:

    def __init__(self, name):
        # Initialize the node and subscribers/publishers
        self.name = name
        self.plot_data_subscriber = rospy.Subscriber(f'{name}plot_data', plot_data, self.plot_data_callback)
        self.current_vel_x = 0.0
        self.current_vel_y = 0.0

        #Set the initials for plotting data
        self.time_data = []
        self.velocity_x_data = []
        self.velocity_y_data = []
        self.velocity_mag_data = []
        self.accel_long_data = []
        self.accel_lat_data = []
        self.accel_mag_data = []
        self.avoidance_data = []
        self.avoid_i = 0

    def plot_data_callback(self, data):
        self.current_vel_x = data.curr_velocity_x
        self.current_vel_y = data.curr_velocity_y
        desired_vel_x = data.des_velocity_x
        desired_vel_y = data.des_velocity_y
        t_iter = 0.5#data.t_iter
        delta = data.delta
        tH = data.t_horizon
        t_i = t_iter
        self.avoid_i = data.avoid_i
        plan_cycle = np.arange(0.0,tH,t_iter)

        # for t_i in plan_cycle[1:]:
        self.current_vel_x = desired_vel_x - math.exp(-t_i/delta)*(desired_vel_x-self.current_vel_x)
        self.current_vel_y = desired_vel_y - math.exp(-t_i/delta)*(desired_vel_y-self.current_vel_y)
        self.update_plotting_data(delta, self.current_vel_x, self.current_vel_y, desired_vel_x, desired_vel_y)



    def update_plotting_data(self, delta, current_vel_x, current_vel_y, desired_vel_x, desired_vel_y):
        # Velocity and Acceleration computing for plotting graphs w.r.t. time.
        a_x = (desired_vel_x - current_vel_x)/delta
        a_y = (desired_vel_y - current_vel_y)/delta
        v = [current_vel_x, current_vel_y]
        a = [a_x, a_y]
        ang = angle_finder(v,a)
        a_mag = np.linalg.norm(a)
        a_long = -a_mag*math.cos(ang)
        a_lat = a_mag*math.sin(ang)
        a_net = np.linalg.norm([a_long, a_lat])
        # a_net = math.sqrt(a_long^2 + a_lat^2)
        self.acceleration = [a_long, a_lat]

        current_time = rospy.Time.now()
        self.time_data.append(current_time.to_sec())
        self.velocity_x_data.append(current_vel_x)
        self.velocity_y_data.append(current_vel_y)
        self.velocity_mag_data.append(np.linalg.norm(v))
        self.accel_long_data.append(a_long)
        self.accel_lat_data.append(a_lat)
        self.accel_mag_data.append(a_net)
        self.avoidance_data.append(self.avoid_i)



if __name__ == '__main__':
    try:
        rospy.init_node('tb3_plot_data_recorder')

        robot_namespace = rospy.get_namespace()

        pass_rate = rospy.Rate(30)

        while not rospy.has_param(robot_namespace+'/ogm_created'):  # untill ogm_created is set turtlebot object is not constructed
            pass_rate.sleep()


        if rospy.has_param(robot_namespace+'/ogm_created'):
            rospy.loginfo('ogm_created parameter has been set')
        else:
            rospy.logwarn('ogm_created parameter is not set')

        data_rec = Recoder(robot_namespace)
        rospy.spin()
        
        plot_params = ['time1','velocity_x1','velocity_y1','velocity_mag1','accel_long1','accel_lat1','accel_mag1', 'avoidance']

        bag_0 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[0] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_1 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[1] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_2 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[2] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_3 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[3] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_4 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[4] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_5 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[5] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_6 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[6] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')
        bag_7 = rosbag.Bag('/home/balu/catkin_ws/src/move_robot/bags/'+plot_params[6] + '_data_' + robot_namespace.replace('/', '') + '.bag', 'w')

        # bags = {bag_0, bag_1,bag_2,bag_3,bag_4,bag_5,bag_6}
        # print(data_rec.velocity_mag_data)
        time_data = Float32MultiArray()
        velocity_x_data = Float32MultiArray()
        velocity_y_data = Float32MultiArray()
        velocity_mag_data = Float32MultiArray()
        accel_long_data = Float32MultiArray()
        accel_lat_data = Float32MultiArray()
        accel_mag_data = Float32MultiArray()
        avoidance_data = Float32MultiArray()

        time_data.data = data_rec.time_data
        velocity_x_data.data = data_rec.velocity_x_data
        velocity_y_data.data = data_rec.velocity_y_data
        velocity_mag_data.data = data_rec.velocity_mag_data
        accel_long_data.data = data_rec.accel_long_data
        accel_lat_data.data = data_rec.accel_lat_data
        accel_mag_data.data = data_rec.accel_mag_data
        avoidance_data.data = data_rec.avoidance_data

        bag_0.write(robot_namespace.replace('/', '')+'_'+plot_params[0]+'_data', time_data)
        bag_1.write(robot_namespace.replace('/', '')+'_'+plot_params[1]+'_data', velocity_x_data)
        bag_2.write(robot_namespace.replace('/', '')+'_'+plot_params[2]+'_data', velocity_y_data)
        bag_3.write(robot_namespace.replace('/', '')+'_'+plot_params[3]+'_data', velocity_mag_data)
        bag_4.write(robot_namespace.replace('/', '')+'_'+plot_params[4]+'_data', accel_long_data)
        bag_5.write(robot_namespace.replace('/', '')+'_'+plot_params[5]+'_data', accel_lat_data)
        bag_6.write(robot_namespace.replace('/', '')+'_'+plot_params[6]+'_data', accel_mag_data)
        bag_7.write(robot_namespace.replace('/', '')+'_'+plot_params[7]+'_data', avoidance_data)
        # print(bag_0)

        
    except rospy.ROSInterruptException:
        rospy.logerr("tb3 plot data recorder failed")
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
