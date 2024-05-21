#!/usr/bin/env python

import numpy as np
import math
from scipy.stats import norm
import sys
SCRIPTS_PATH = '/home/ubuntu/catkin_ws/src/move_robot/scripts'
sys.path.insert(0, SCRIPTS_PATH)

def  angle_finder(v,v_des):

    cur_ang = math.atan2(v[1],v[0]); # current velocity angle
    if (cur_ang<0):
       cur_ang = cur_ang+2*math.pi

    des_ang = math.atan2(v_des[1],v_des[0]); # desired velocity angle
    if (des_ang<0):
       des_ang = des_ang+2*math.pi
    
    ang = (des_ang - cur_ang + math.pi )%2*math.pi + math.pi; 
    if (ang>2*math.pi):
        ang = ang - 2*math.pi

    return ang

def find_los_angle(x2,y2,x1,y1,yaw):
    dx = x2 - x1
    dy = y2 - y1
    distance = math.sqrt(dx**2 + dy**2)
    theta = math.atan2(dy,dx)

    if (theta>1.57 and yaw<-1.57) or (yaw>1.57 and theta<-1.57):
        if theta<0:
            theta = theta + 2*np.pi
        if yaw<0:
            yaw = yaw + 2*np.pi
    # print("theta_goal_1",theta*180/np.pi)
    # print("yaw",theta*180/np.pi)
    angle = (theta - yaw)
    # print("angle_goal",angle*180/np.pi)
    # print("theta, yaw, angle", theta*(180/np.pi), yaw*(180/np.pi), angle*(180/np.pi))
    return distance, angle, theta

def obstacles_under_horizon(agent_pos, obstacles):
    obst_tag = []
    tag = []
    d_min = 100000000000
    agent_x = agent_pos.pose.pose.position.x
    agent_y = agent_pos.pose.pose.position.y
    
    for i in range(len(obstacles)):
        tag = obstacles[i]
        obst_tag.append(tag)
        obs_x = obstacles[i].center.x
        obs_y = obstacles[i].center.y
        d = math.sqrt((obs_x- agent_x)**2 + (obs_y - agent_y)**2)
        if (d_min > d):
            d_min = d

    return obst_tag, d_min

def get_conflict_region(R0,R1,R2,R3,R4,R5,angle):
    region = 0

    if ((angle >= R0) and (angle <= R1)) or ((angle >= R4) and (angle <= R5)):
        region = 0
    elif (angle > R1) and (angle <= R2):
        region = 1
    elif (angle > R2) and (angle < R3):
        region = 2
    else:
        region = 3

    return region

def get_collision_course_region(R0_a,R1_a,R2_a,R3_a,alpha_cc,agent_hed):

    if ((agent_hed >= R0_a) and (agent_hed <= alpha_cc+R1_a)) or ((agent_hed >= alpha_cc+R2_a) and (agent_hed <= R3_a)):
        coll_course = 1 # IN. The heading direction of the agent is within the collision course triangle
    else:
        coll_course = 0 # OUT
    # print("coll_course", coll_course)
    return coll_course


def Encounter_type_2(obst_tags, alpha_i, R_d, agent_pos, agent_vel_x, agent_vel_y, map_x_len,map_y_len):
    ROW = []
    ROW1 = 0
    right_of_way = False
    stat_obst = True
    alpha_j = None

    agent_x = agent_pos.pose.pose.position.x
    agent_y = agent_pos.pose.pose.position.y

    # #Our Angular Regions
    # R0 = 0*np.pi/180
    # R1 = 45*np.pi/180
    # R2 = 135*np.pi/180
    # R3 = 225*np.pi/180
    # R4 = 315*np.pi/180
    # R5 = 360*np.pi/180

    #COLREG Angular Regions
    R0 = 0*np.pi/180
    R1 = 45*np.pi/180
    R2 = 115.5*np.pi/180
    R3 = 247.5*np.pi/180
    R4 = 330*np.pi/180
    R5 = 360*np.pi/180

    R0_a = 0*np.pi/180
    R1_a = 45*np.pi/180
    R2_a = 315*np.pi/180
    R3_a = 360*np.pi/180
    # print("problem 1")

    # x = 0 # if dist >= 2*R_d, then x=1
    # RoW_Matrix = np.array([[x, 1, 1, x],     # ([[x, 1, 1, x],
                        #    [0, x, 1, 0]])    #   [0, x, 1, 0]])
                                            
    # print("problem 2")
    # print("alpha_i_1", alpha_i*(180/np.pi))
    if alpha_i < 0:
            alpha_i = 2*np.pi + alpha_i
    # print("alpha_i_2", alpha_i*(180/np.pi))

    for j in range(len(obst_tags)):
        if (np.abs(obst_tags[j].center.x - agent_x) <= map_x_len/2) and (np.abs(obst_tags[j].center.y - agent_y) <= map_y_len/2):
            # print("problem 3")
            obs_x = obst_tags[j].center.x
            obs_y = obst_tags[j].center.y
            obs_vel_x = obst_tags[j].velocity.x
            obs_vel_y = obst_tags[j].velocity.y
            # print("problem 4")
            theta = math.atan2((obs_y-agent_y), (obs_x-agent_x))
            # print("theta_1", theta*(180/np.pi))
            if theta < 0:
                theta = 2*np.pi + theta
            # print("problem 5")
            # print("obs_vel_x", obs_vel_x)
            # print("agent_vel_x", agent_vel_x)
            alpha_j = math.atan2((agent_vel_y + obs_vel_y),(agent_vel_x + obs_vel_x)) #obs_vel_y and obs_vel_x are the relative velocitieas on X and Y direction
            # print("alpha_j_1", alpha_j*(180/np.pi))
            if alpha_j < 0:
                alpha_j = 2*np.pi + alpha_j
            # print("problem 6")
            beta = ((theta - alpha_i + np.pi)%(2*np.pi)) + (np.pi)
            # print("beta_1", beta*(180/np.pi))
            if beta > 2*np.pi:
                beta = beta - 2*np.pi
            # print("problem 7")
            alpha = ((alpha_j - alpha_i + np.pi)%(2*np.pi)) + (np.pi)
            # print("alpha_1", alpha*(180/np.pi))
            if alpha > 2*np.pi:
                alpha = alpha - 2*np.pi
            # print("problem 8")
            mu = (np.linalg.norm([agent_vel_x,agent_vel_y])/np.linalg.norm([(agent_vel_y + obs_vel_y),(agent_vel_x + obs_vel_x)]))
            # print("problem 9")
            # print("mu", mu)

            if (-1<mu<1):
                alpha_region = 1 
            else:
                alpha_cc = theta + math.asin((1/mu)*math.sin(alpha_j - theta))

                if alpha_cc < 0:
                    alpha_cc = 2*np.pi + alpha_j
                elif alpha_cc > 2*np.pi:
                    alpha_cc = alpha_cc - 2*np.pi

                # print("alpha_cc", alpha_cc*(180/np.pi))
                alpha_region = get_collision_course_region(R0_a,R1_a,R2_a,R3_a,alpha_cc,alpha_i)

            # print("problem 11")
            beta_region = get_conflict_region(R0,R1,R2,R3,R4,R5,beta)
            # # print("problem 12")
            # # print("problem 13")
            # print("beta_2", beta*(180/np.pi))
            # # print("alpha_j_2", alpha_j*(180/np.pi))
            # # print("theta_2", theta*(180/np.pi))
               
            # # print("alpha_2", alpha*(180/np.pi))
           
            

            dist_agents = np.sqrt((agent_x-obs_x)**2+(agent_y-obs_y)**2)
            # print("dist_agents",dist_agents)

            # if ((abs(agent_vel_x + obs_vel_x) <= 0.01) and (abs(agent_vel_y + obs_vel_y) <= 0.01)):
            if ((abs(obs_vel_x) <= 0.01) and (abs(obs_vel_y) <= 0.01)):
                # print("velocity of obstacle is nearly zero")
                if ((beta_region == 2) or (dist_agents >= 2*np.sqrt(2)*R_d)): 
                    alpha_region = 1
                    stat_obst = True #Static obstcale but no problem 
                    # print("Static obstcale but no problem")
                else:
                    stat_obst = False
                    # print("Found static obstacle(s)")

            # print("alpha_region",alpha_region)
            # print("beta_region",beta_region)

            if (dist_agents >= 2*np.sqrt(2)*R_d):
                x = 1
            else:
                x = 0
            
            RoW_Matrix = np.array([[x, 1, 1, x], 
                                   [0, x, 1, 0]])
            
            ROW1 = RoW_Matrix[alpha_region,beta_region]

            # print("problem 14")
            # print("ROW1", ROW1)
            # print("safe dist:", 2*np.sqrt(2)*R_d)

            if (dist_agents < 2*np.sqrt(2)*R_d):
                # print("distance between agents is smaller than safer one:", dist_agents)
                ROW1 = 0
                if (beta_region == 2):
                    ROW1 = 1

            ROW.append(ROW1)
            # print("ROW", ROW)
            # print("agent_vel_x",agent_vel_x)
            # print("obs_vel_x",obs_vel_x)

            

    
    if (len(ROW)==np.sum(ROW)):
        right_of_way = True
       
    # print("Number of obstacles within the range of map are", (len(ROW)))
    # print("ROW", ROW)
    return right_of_way,stat_obst


def Encounter_type(obst_tags, alpha_i, R_d, agent_pos, agent_vel_x, agent_vel_y, map_x_len,map_y_len):
    ROW = []
    ROW1 = 0
    right_of_way = False
    stat_obst = True
    alpha_j = None

    agent_x = agent_pos.pose.pose.position.x
    agent_y = agent_pos.pose.pose.position.y

    # #Our Angular Regions
    # R0 = 0*np.pi/180
    # R1 = 45*np.pi/180
    # R2 = 135*np.pi/180
    # R3 = 225*np.pi/180
    # R4 = 315*np.pi/180
    # R5 = 360*np.pi/180

    #COLREG Angular Regions
    R0 = 0*np.pi/180
    R1 = 45*np.pi/180
    R2 = 115.5*np.pi/180
    R3 = 247.5*np.pi/180
    R4 = 330*np.pi/180
    R5 = 360*np.pi/180

    RoW_Matrix = np.array([[0, 1, 1, 1],   
                           [0, 1, 1, 0],
                           [0, 1, 1, 1],
                           [0, 1, 1, 1]])
    # print("alpha_i_1", alpha_i*(180/np.pi))
    if alpha_i < 0:
        alpha_i = 2*np.pi + alpha_i
    # print("alpha_i_2", alpha_i*(180/np.pi))
    for j in range(len(obst_tags)):
        if (np.abs(obst_tags[j].center.x - agent_x) <= map_x_len/2) and (np.abs(obst_tags[j].center.y - agent_y) <= map_y_len/2):
            obs_x = obst_tags[j].center.x
            obs_y = obst_tags[j].center.y
            obs_vel_x = obst_tags[j].velocity.x
            obs_vel_y = obst_tags[j].velocity.y
            
            theta = math.atan2((obs_y-agent_y), (obs_x-agent_x))
            # print("theta_1", theta*(180/np.pi))
            if theta < 0:
                theta = 2*np.pi + theta
            
            # print("obs_vel_x", obs_vel_x)
            # print("agent_vel_x", agent_vel_x)
            alpha_j = math.atan2((agent_vel_y + obs_vel_y),(agent_vel_x + obs_vel_x)) #obs_vel_y and obs_vel_x are the relative velocitieas on X and Y direction
            # print("alpha_j_1", alpha_j*(180/np.pi))
            if alpha_j < 0:
                alpha_j = 2*np.pi + alpha_j
            
            beta = ((theta - alpha_i + np.pi)%(2*np.pi)) + (np.pi)
            # print("beta_1", beta*(180/np.pi))
            if beta > 2*np.pi:
                beta = beta - 2*np.pi
            
            alpha = ((alpha_j - alpha_i + np.pi)%(2*np.pi)) + (np.pi)
            # print("alpha_1", alpha*(180/np.pi))
            if alpha > 2*np.pi:
                alpha = alpha - 2*np.pi

            beta_region = get_conflict_region(R0,R1,R2,R3,R4,R5,beta)
            alpha_region = get_conflict_region(R0,R1,R2,R3,R4,R5,alpha)

            # # print("theta_2", theta*(180/np.pi))
            # # print("alpha_j_2", alpha_j*(180/np.pi))
            # # print("alpha_2", alpha*(180/np.pi))
            # # print("beta_2", beta*(180/np.pi))
            # print("alpha_region",alpha_region)
            # print("beta_region",beta_region)
            dist_agents = np.sqrt((agent_x-obs_x)**2+(agent_y-obs_y)**2)
            # print("dist_agents",dist_agents)
            ROW1 = RoW_Matrix[alpha_region,beta_region]
            # print("ROW1", ROW1)
            # print("safe dist:", 2*np.sqrt(2)*R_d)

            if ((dist_agents < 2*np.sqrt(2)*R_d) and (beta_region == 3) and ((alpha_region == 0) or (alpha_region == 2))):
                # print("distance between agents is smaller than safer one:", dist_agents)
                ROW1 = 0

            ROW.append(ROW1)
            # print("ROW", ROW)
            # print("agent_vel_x",agent_vel_x)
            # print("obs_vel_x",obs_vel_x)

            # if ((abs(agent_vel_x + obs_vel_x) <= 0.01) and (abs(agent_vel_y + obs_vel_y) <= 0.01)):
            if ((abs(obs_vel_x) <= 0.01) and (abs(obs_vel_y) <= 0.01)):
                print("velocity of obstacle is nearly zero")
                if ((beta_region == 2) or (dist_agents >= 2*np.sqrt(2)*R_d)): 
                    stat_obst = True #Static obstcale but no problem 
                    print("Static obstcale but no problem")
                else:
                    stat_obst = False
                    print("Found static obstacle(s)")

    
    if (len(ROW)==np.sum(ROW)):
        right_of_way = True

    print("ROW", ROW)
    # print("Number of obstacles within the range of map are", (len(ROW)))
    return right_of_way,stat_obst

def admissible_vel(a_max,lin_speed,vel_x, vel_y,yaw,del_t,goal_x,goal_y,pos_x,pos_y):
    v = lin_speed
    v_max = 0.22

    if v < 0.01: #when turtlebot has only angular velocity, then linear velocity will be zero. So, we cannot create "k" array. 
        v += 0.001

    v_n = 8 #7 #number of velocity points are v_n^2
    # a = np.arange(-a_max, a_max, 10)
    a_len = (2*a_max)/v_n
    a = np.arange(-a_max, a_max, a_len)
    a_x = []
    a_y = []
    #w = np.arange(-0.5,0.5,10)
    #k = math.tan(w*0.01)/0.006
    k_len = (2*a_max/v**2)/v_n
    k_lim = a_max/v**2
    k = np.arange(-k_lim, k_lim, k_len)
    R = np.array([[math.cos(yaw), -math.sin(yaw)],
                  [math.sin(yaw), math.cos(yaw)]])
    
    # create a meshgrid of the two arrays
    A, B = np.meshgrid(k, a)
    # stack the two arrays to create a matrix
    P = np.column_stack((B.flatten(), A.flatten()))

    for i in range(0,int(len(P))):
        a_x.append(P[i][0])
        a_y.append((v**2)*P[i][1])

    A = np.dot(R, np.array([a_x,
                            a_y]))

    #reachable velocities
    v_x = A[0]*del_t 
    v_y = A[1]*del_t

    dis, theta, ang_temp = find_los_angle(goal_x,goal_y,pos_x,pos_y, yaw)

    v_goal = [math.cos(theta), math.sin(theta)]
    # v_curr = [vel_x, vel_y]

    right_v_x = []
    right_v_y = []
    min_goal = []

    #get a new RVS that includes velocities only towards right of the current velocity
    for i in range(len(v_x)):
        alpha_i_rvs  = math.atan2(v_y[i],v_x[i]) 
        if (0<= (yaw-alpha_i_rvs) <= np.pi):
            right_v_x.append(v_x[i])
            right_v_y.append(v_y[i])
            min_goal.append(np.dot(v_goal, [v_x[i], v_y[i]]))

    # print("Right RVS_x",right_v_x)
    rvs_v_x = []
    rvs_v_y = []

    #ordered Reachable Velocity Set 
    for j in range(len(min_goal)):
        min_index = min_goal.index(max(min_goal))
        rvs_v_x.append(right_v_x[min_index])
        rvs_v_y.append(right_v_y[min_index])
        min_goal[min_index] = -1
    # print("rvs_x", rvs_v_x)
    # print("rvs_y", rvs_v_y)
    # np.flipud(rvs_v_x)
    # print("RVS", np.array([rvs_v_x, rvs_v_y]))
    # print("Flipped RVS", np.array([np.flipud(rvs_v_x), np.flipud(rvs_v_y)]))
    # np.flipud(rvs_v_y)

    # rvs_v_x11 = rvs_v_x[int(len(rvs_v_x)/2):]
    # ind1 = rvs_v_x.index(rvs_v_x[int(len(rvs_v_x)/2)])-1
    # rvs_v_x12 = np.flipud(rvs_v_x[0:ind1+1])
    # rvs_v_x1 = [x for n in (rvs_v_x11,rvs_v_x12) for x in n]

    # rvs_v_y11 = rvs_v_y[int(len(rvs_v_y)/2):]
    # ind2 = rvs_v_x.index(rvs_v_x[int(len(rvs_v_x)/2)])-1
    # rvs_v_y12 = np.flipud(rvs_v_y[0:ind2+1])
    # rvs_v_y1 = [y for m in (rvs_v_y11,rvs_v_y12) for y in m]

    return np.array([np.flipud(rvs_v_x), np.flipud(rvs_v_y)])
    # return np.array([rvs_v_x, rvs_v_y])
    # return np.array([rvs_v_x1, rvs_v_y1])


def compute_pdf(vel,vel_var):
    mu = vel
    sigma = np.sqrt(vel_var)
    n = 3 #number of samples
    x = np.linspace((mu-3*sigma),(mu+3*sigma), n)
    pdf = []
    for i in x:
        prob = norm.pdf((i-mu)/sigma, loc = mu, scale = sigma)
        pdf.append(prob)
    # print(pdf)
    # print(np.sum(pdf)*(2*3*sigma)/n)

    return x, pdf

def compute_avo(vb_x,vb_y,va_x,va_y,plan_cycle,delta,pb_x,pb_y,pa_x, pa_y,w,V_des_x,V_des_y):
    for ti in plan_cycle[1:]:
        square_center_x = (delta*(math.exp(-ti/delta)-1)*(vb_x)-(pa_x- pb_x))/(ti+delta*(math.exp(-ti/delta)-1))
        square_center_y = (delta*(math.exp(-ti/delta)-1)*(vb_y)-(pa_y- pb_y))/(ti+delta*(math.exp(-ti/delta)-1))
        square_side = (math.sqrt(2)*w)/((ti+delta*(math.exp(-ti/delta)-1)))

        x_min = square_center_x-square_side/2
        y_min = square_center_y-square_side/2

        x_max = square_center_x+square_side/2
        y_max = square_center_y+square_side/2

        acc_obs_x = 0 # need to find the acceleration of the obstcles
        acc_obs_y = 0
        vb_x_t = vb_x + delta*(math.exp(-ti/delta)-1)*acc_obs_x
        vb_y_t = vb_y + delta*(math.exp(-ti/delta)-1)*acc_obs_y

        if ((V_des_x<=(vb_x_t + x_max)) and (V_des_x>=(vb_x_t+x_min)) and (V_des_y>=(vb_y_t+y_min)) and (V_des_y<=(vb_y_t+y_max))): 
            I_AVO = 1
            break
        else:
            I_AVO = 0
    # print("I_AVO",I_AVO)

    return I_AVO

def PVO_Controller(ogm,pos_x,pos_y,current_vel_x,current_vel_y,yaw,tH,a_max,del_t,goal_x, goal_y,R_d, vel_var):
    v_x_des = 0.0
    v_y_des = 0.0
    w = 2*0.12 #2*R_d #10.5 cm is the actual radius of tb_3 burger model
    plan_cycle = np.arange(0.0,tH,0.13)
    lin_speed = np.linalg.norm([current_vel_x,current_vel_y])
    RVS = admissible_vel(a_max,lin_speed,current_vel_x,current_vel_y, yaw,del_t,goal_x,goal_y,pos_x,pos_y)
    rvs_pub = RVS
    # print("rvs_pub", rvs_pub)
    prob_safe = 0.1 #0.2
    #for each velocity of ordered RVS find the probability of colliosion
    for vi in range(int((RVS.size)/2)):
        # print("planned_vi_1",RVS[0][vi],RVS[1][vi])
        prob_vi_des_ti = 0
        # for ti in plan_cycle[1:]:
        prob_vi_des_ti_tH = 0
        prob_col_vi_des_comp = 1
        for row_i in range(len(ogm[0])):
            for col_i in range(len(ogm[0][0])):
                # print(ogm[0])
                # print("row_i", row_i,"col_i",col_i)
                if (ogm[0][row_i][col_i] > 0.6):
                    # print("occupancy", ogm[0][row_i][col_i])
                    prob_col_vi_des_obs = 0

                    vel_x_range, obs_vel_x_pdf = compute_pdf((ogm[1][row_i][col_i]-current_vel_x),vel_var) #(relative velocity - agent_current_velocity)
                    vel_y_range, obs_vel_y_pdf = compute_pdf((ogm[2][row_i][col_i]-current_vel_y),vel_var)
                    obs_pos_x = ogm[3][row_i][col_i]
                    obs_pos_y = ogm[4][row_i][col_i]

                    prob_col_vi_des_obs_vel = 0
                    vel_i_n = 0
                    # for vel_i in range(len(obs_vel_x_pdf)):
                    for vel_j in range(len(obs_vel_y_pdf)):
                        I_AVO = compute_avo(vel_x_range[vel_j],vel_y_range[vel_j],current_vel_x,current_vel_y,plan_cycle,del_t,obs_pos_x,obs_pos_y,pos_x,pos_y,w,RVS[0][vi],RVS[1][vi])
                        prob_col_vi_des_obs_vel = ogm[0][row_i][col_i]*obs_vel_x_pdf[vel_j]*obs_vel_y_pdf[vel_j]*I_AVO
                        # print("prob_col_vi_des_obs_vel", prob_col_vi_des_obs_vel)
                        # print("I_AVO",I_AVO)
                        prob_col_vi_des_obs += prob_col_vi_des_obs_vel
                        # print("prob_col_vi_des_obs", prob_col_vi_des_obs)
                    
                    prob_col_vi_des_comp *= (1-prob_col_vi_des_obs)
                    # print("prob_col_vi_des_comp", prob_col_vi_des_comp)

        prob_vi_des_ti_tH = 1-prob_col_vi_des_comp
        # print("prob_vi_des_ti_tH", prob_vi_des_ti_tH)
        if (prob_vi_des_ti_tH <= prob_safe):
            # print("planned_vi_2",RVS[0][vi],RVS[1][vi])
            v_x_des = RVS[0][vi]
            v_y_des = RVS[1][vi]
            break

    return v_x_des, v_y_des, rvs_pub  

# def compute_avo(vb_x,vb_y,va_x,va_y,ti,delta,pb_x,pb_y,pa_x, pa_y,w,V_des_x,V_des_y):
#     square_center_x = (delta*(math.exp(-ti/delta)-1)*(vb_x)-(pa_x- pb_x))/(ti+delta*(math.exp(-ti/delta)-1))
#     square_center_y = (delta*(math.exp(-ti/delta)-1)*(vb_y)-(pa_y- pb_y))/(ti+delta*(math.exp(-ti/delta)-1))
#     square_side = (math.sqrt(2)*w)/((ti+delta*(math.exp(-ti/delta)-1)))

#     x_min = square_center_x-square_side/2
#     y_min = square_center_y-square_side/2

#     x_max = square_center_x+square_side/2
#     y_max = square_center_y+square_side/2

#     acc_obs_x = 0 # need to find the acceleration of the obstcles
#     acc_obs_y = 0
#     vb_x_t = vb_x + delta*(math.exp(-ti/delta)-1)*acc_obs_x
#     vb_y_t = vb_y + delta*(math.exp(-ti/delta)-1)*acc_obs_y

#     if ((V_des_x<=(vb_x_t + x_max)) and (V_des_x>=(vb_x_t+x_min)) and (V_des_y>=(vb_y_t+y_min)) and (V_des_y<=(vb_y_t+y_max))): 
#         I_AVO = 1
#     else:
#         I_AVO = 0

#     return I_AVO


# def PVO_Controller(ogm,pos_x,pos_y,current_vel_x,current_vel_y,yaw,tH,a_max,del_t,goal_x, goal_y,R_d, vel_var):
#     v_x_des = 0.0
#     v_y_des = 0.0
#     w = 2*0.11 #2*R_d #10.5 cm is the actual radius of tb_3 burger model
#     plan_cycle = np.arange(0.0,tH,0.13)
#     lin_speed = np.linalg.norm([current_vel_x,current_vel_y])
#     RVS = admissible_vel(a_max,lin_speed,current_vel_x,current_vel_y, yaw,del_t,goal_x,goal_y,pos_x,pos_y)
#     rvs_pub = RVS
#     prob_safe = 0.0
#     #for each velocity of ordered RVS find the probability of colliosion
#     for vi in range(int((RVS.size)/2)):
#         prob_vi_des_ti = 0
#         for ti in plan_cycle[1:]:
#             prob_vi_des_ti_tH = 0
#             prob_col_vi_des_comp = 1
#             for row_i in range(len(ogm[0])):
#                 for col_i in range(len(ogm[0][0])):
#                     # print(ogm[0])
#                     # print("row_i", row_i,"col_i",col_i)
#                     if (ogm[0][row_i][col_i] > 0.6):
#                         # print("occupancy", ogm[0][row_i][col_i])
#                         prob_col_vi_des_obs = 0

#                         vel_x_range, obs_vel_x_pdf = compute_pdf((ogm[1][row_i][col_i]-current_vel_x),vel_var) #(relative velocity - agent_current_velocity)
#                         vel_y_range, obs_vel_y_pdf = compute_pdf((ogm[2][row_i][col_i]-current_vel_y),vel_var)
#                         obs_pos_x = ogm[3][row_i][col_i]
#                         obs_pos_y = ogm[4][row_i][col_i]

#                         prob_col_vi_des_obs_vel = 0

#                         for vel_i in range(len(obs_vel_x_pdf)):
#                             for vel_j in range(len(obs_vel_y_pdf)):
#                                 I_AVO = compute_avo(vel_x_range[vel_i],vel_y_range[vel_j],current_vel_x,current_vel_y,ti,del_t,obs_pos_x,obs_pos_y,pos_x,pos_y,w,RVS[0][vi],RVS[1][vi])
#                                 prob_col_vi_des_obs_vel = ogm[0][row_i][col_i]*obs_vel_x_pdf[vel_i]*obs_vel_y_pdf[vel_j]*I_AVO
#                                 prob_col_vi_des_obs += prob_col_vi_des_obs_vel

                        
#                         prob_col_vi_des_comp *= (1-prob_col_vi_des_obs)

#             prob_vi_des_ti = 1-prob_col_vi_des_comp
#             prob_vi_des_ti_tH += (1-prob_vi_des_ti_tH)*prob_vi_des_ti

#         if (prob_vi_des_ti_tH <= prob_safe):
#             v_x_des = RVS[0][vi]
#             v_y_des = RVS[1][vi]

#     return v_x_des, v_y_des, rvs_pub
             
