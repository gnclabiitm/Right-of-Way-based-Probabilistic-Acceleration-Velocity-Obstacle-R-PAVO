#!/usr/bin/env python3

import rospy
import numpy as np
import rosbag
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

rospy.init_node('bag_file_plotter')

# List of robot namespaces
robot_namespaces = ['tb3_0', 'tb3_1','tb3_2','tb3_3','tb3_4']  # Add more namespaces if needed

# Function to read and extract data from bag file
def read_bag_file(bag_filename, topic):
    data_values = []

    bag = rosbag.Bag(bag_filename, 'r')
    # print('bag', bag)
    for topic, msg, t in bag.read_messages(topics=[topic]):
        data_values = msg.data
        # print("flag 1", data_values)
    bag.close()
    # print("flag 2", data_values)
    return data_values

# Plotting function
def plot_data(time_data, data_values, ylabel):
    plt.plot(time_data, data_values)
    plt.xlabel('Time (s)')
    plt.ylabel(ylabel)
    plt.grid(True)


# Read and plot velocity data for each robot
plot_params = ['velocity_x_data_','velocity_y_data_', 'velocity_mag_data_', 'avoidance_data_', 'avoidance_data_', 'avoidance_data_', 'accel_long_data_', 'accel_lat_data_', 'accel_mag_data_']
for param in plot_params:
    # plt.figure(figsize=(10, 5))  # Create a new figure for plots
    plt.subplot(3,3,plot_params.index(param)+1) #to get single plot for each, comment this and uncomment the top line plt.figure() 
    for namespace in robot_namespaces:
        if (namespace == 'tb3_3'):
            time_bag_filename = '/home/balu/catkin_ws/src/move_robot/bags/'+'time_data_' + namespace + '.bag'
            bag_filename = '/home/balu/catkin_ws/src/move_robot/bags/'+ param + namespace + '.bag'
            # print(bag_filename)
            time_data = read_bag_file(time_bag_filename, namespace+'_time_data')
            # print(namespace+'_'+param)
            values = read_bag_file(bag_filename, namespace+'_'+param[:-1])
            # print(time_data)
            # print("flag 3",values

            plot_data(time_data, values, param.replace('_data_', ""))
            plt.legend(robot_namespaces)

    plt.title(param.replace('_data_', "")+' vs Time')
plt.tight_layout()
plt.show()


#------------------------------------- to plot single images of each parameter----------------------------------------------------------------------

# import rospy
# import rosbag
# import matplotlib.pyplot as plt

# rospy.init_node('bag_file_plotter')

# # List of robot namespaces
# robot_namespaces = ['tb3_0', 'tb3_1','tb3_2','tb3_3','tb3_4']  # Add more namespaces if needed

# # Function to read and extract data from bag file
# def read_bag_file(bag_filename, topic):
#     data_values = []

#     bag = rosbag.Bag(bag_filename, 'r')
#     # print('bag', bag)
#     for topic, msg, t in bag.read_messages(topics=[topic]):
#         data_values = msg.data
#         # print("flag 1", data_values)
#     bag.close()
#     # print("flag 2", data_values)
#     return data_values

# # Plotting function
# def plot_data(time_data, data_values, ylabel):
#     plt.plot(time_data, data_values)
#     plt.xlabel('Time (s)')
#     plt.ylabel(ylabel)
#     plt.grid(True)

# # Read and plot velocity data for each robot
# plot_params = ['velocity_x_data_', 'velocity_y_data_', 'velocity_mag_data_', 'accel_lat_data_', 'accel_long_data_', 'accel_mag_data_']
# for param in plot_params:
#     plt.figure(figsize=(10, 5))  # Create a new figure for plots

#     for namespace in robot_namespaces:
#         time_bag_filename = '/home/balu/catkin_ws/src/move_robot/bags/'+'time_data_' + namespace + '.bag'
#         bag_filename = '/home/balu/catkin_ws/src/move_robot/bags/'+ param + namespace + '.bag'
#         # print(bag_filename)
#         time_data = read_bag_file(time_bag_filename, namespace+'_time_data')
#         # print(namespace+'_'+param)
#         values = read_bag_file(bag_filename, namespace+'_'+param[:-1])
#         # print(time_data)
#         # print("flag 3",values)

#         plot_data(time_data, values, param.replace('_data_', ""))
#         plt.legend(robot_namespaces)

#     plt.title(param.replace('_data_', "")+' vs Time')
#     plt.show()
