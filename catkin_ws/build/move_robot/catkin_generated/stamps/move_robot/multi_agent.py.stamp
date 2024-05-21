#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from move_robot.srv import *
from move_robot.srv import OGM
from grid_map_msgs.msg import GridMap
 
cmd_vel_pub = [None, None, None, None, None]
kp = 0.25
global n

class Agent:
    def __init__(self, id, pos, goal, ori):
        self.id = id
        self.position = pos
        self.goal = goal
        self.orientation = ori

    def odometryCallback(self,msg):
        roll = pitch = yaw = 0.0
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        ornt_q = msg.pose.pose.orientation
        ornt_list = [ornt_q.x, ornt_q.y, ornt_q.z, ornt_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(ornt_list)
        self.orientation = yaw

    def ogmCallback(self,msg):
        self.ogm_positions_x = msg.info.pose.position.x 
        self.ogm_positions_y = msg.info.pose.position.y 
        self.ogm_positions_layer = msg.layers[0]
        self.ogm_prev_positions_layer = msg.layers[1]
        self.ogm_velocities_layer = msg.layers[2]
        self.ogm_positions = msg.data[0]
        self.ogm_prev_positions = msg.data[1]
        self.ogm_velocities = msg.data[2]
        print(self.id)
        print(self.ogm_positions_x)
        print(self.ogm_positions_layer)
        # print(self.ogm_positions)

def find_velocity(agent):
    global kp
    #Calculating the distance and angle between the robot's current and goal position
    rospy.loginfo("if activated \n")
    dx = agent.goal[0] - agent.position[0]
    dy = agent.goal[1] - agent.position[1] 
    distance = math.sqrt(dx**2 + dy**2)
    theta = math.atan2(dy,dx)
    angle = (theta - agent.orientation)

    #Calculating the linear and angular velocities based on the calculated distance and angle        
    if angle>0.1:
        angular_vel = kp*angle
        linear_vel = 0.1 
    elif angle<-0.1:
        angular_vel = -kp*angle 
        linear_vel = 0.1 
    else:
        angular_vel = 0.0
        linear_vel = 0.22

    return linear_vel, angular_vel

def goals_reached(agents, goal_tol, turtle_ids, n):
    global cmd_vel_pub
    reach_bits = [0,0,0,0,0]
    
    for i_dis in range(n):
        dist = [agents[i_dis].position, agents[i_dis].goal]
        d = math.sqrt((dist[1][0]-dist[0][0])**2 + (dist[1][1]-dist[0][1])**2)

        if d>goal_tol:
            reach_bit = 0
            [linear_vel, angular_vel] = find_velocity(agents[i_dis])
        else:
            reach_bit = 1
            linear_vel = 0.0
            angular_vel = 0.0
            
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel

        reach_bits[i_dis] = reach_bit

        cmd_vel_pub[i_dis].publish(twist_msg)
        #print('awaken')
  
    if sum(reach_bits) == n:
        Reach = True
    else:
        Reach = False

    return Reach        


def main():
    
    rospy.init_node('multibots_move_to_goal', anonymous=True)
    r = rospy.Rate(30) # 30hz 
    global n
    n = 5
    goal_tol = 0.1
    turtle_ids = ['tb3_0','tb3_1','tb3_2','tb3_3','tb3_4']
    start_pos = [[-4,-1], [4,-1], [0.5,3], [1.5,1], [3.5,2.5]]
    goal_pos = [[5,5], [6,6], [7,7], [8,8], [9,9]]
    start_ori = [1.57,1.57,0,0,1.07]
    agents = [[],[],[],[],[]]
    tb3_odom = 'tb3_odom'
    tb3_vel = 'tb3_vel'
    ogm_topic = 'ogm_topic'
    for i in range(n):
        node_num = str(i)
        agents[i] = Agent('None',[0,0],[0,0],0)
        agents[i].id = turtle_ids[i]
        agents[i].position = start_pos[i]
        agents[i].goal = goal_pos[i]
        agents[i].orientation = start_ori[i]

        tb3_odom = tb3_odom.replace(tb3_odom,turtle_ids[i]+'/odom')
        rospy.Subscriber(tb3_odom, Odometry, agents[i].odometryCallback)
        print(tb3_odom)
        
        rospy.wait_for_service('get_OGM')
        ogm_client = rospy.ServiceProxy('get_OGM', OGM)
        ogm = ogm_client(turtle_ids[i], node_num)
        print("came till here")

        ogm_topic = ogm_topic.replace(ogm_topic,"tb3_grid_map_topic_"+str(i))
        print(ogm_topic)
        rospy.Subscriber(ogm_topic, GridMap, agents[i].ogmCallback)
        print("ogm data we got")

        tb3_vel = tb3_vel.replace(tb3_vel,turtle_ids[i]+'/cmd_vel')
        cmd_vel_pub[i] = rospy.Publisher(tb3_vel, Twist, queue_size=10)
        #rospy.loginfo(cmd_vel_pub[i])
        

    #Loop to run the robot towards the goal untill it reaches or if terminal shutdown is done
    while not rospy.is_shutdown():
        #Check if the goal is reached (within the tolerance range)
        Reach = goals_reached(agents, goal_tol, turtle_ids, n)    
        
        if Reach:
            rospy.loginfo("Goal position reached") 
            break #come out of the loop and send zero velocities

        if rospy.is_shutdown():
            break
        r.sleep()
        rospy.spin()
    

if __name__== '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass