#!/usr/bin/python

import rospy
import math as m
import sys
import numpy as np
from random import random
from random import randint
from std_msgs.msg import String
from morph_msg.msg import BroadCast
from geometry_msgs.msg import Point
from im2goal import goal_points
from gazebo_msgs.srv import SetJointProperties, SetJointPropertiesRequest

global current_robots, velocity
current_robots = None
velocity = 3.454

def twoWheelRobotCallback(rcvd_msg):
    global current_robots
    # current_robots.index = rcvd_msg.index
    # current_robots.x = rcvd_msg.x
    # current_robots.y = rcvd_msg.y
    # current_robots.orientation = rcvd_msg.orientation
    # current_robots.left_wheel_vel = rcvd_msg.left_wheel_vel
    # current_robots.right_wheel_vel = rcvd_msg.right_wheel_vel
    # last_check = rospy.get_time()
    current_robots = rcvd_msg
        
def reducedist2goal(pos, point, goal):
    curr_dist = abs(pos.x - goal.x) + abs(pos.y - goal.y)
    next_dist = abs(point.x - goal.x) + abs(point.y - goal.y)
    if next_dist < curr_dist:
        return True
    return False

def findMinMsg(msg_buff):
    min_hop = float(inf)
    for i in msg_buff:
        if current_robots.hop[i] < min_hop:
            min_hop = current_robots.hop[i]
            min_msg = i
    return min_msg

def goalPoint(i):
    global goals
    for x in goals:
        if x[0] == i.x and x[1] == i.y:
            return True
    return False 
    
def goalAssigned(msg_buff, goal):
    for i in msg_buff:
        if current_robots.T[i] == goal:
            return True
    return False 

def turn(vr, vl, t):
    if rospy.get_time() - turn_time[num] < t:
        wheel_vel[num][0], wheel_vel[num][1] = vr, vl
        done = False
    else:
        wheel_vel[num][0], wheel_vel[num][1] = 0.0, 0.0
        done = True
    return done

def moveRobot(a, b, num):
    global velocity

    if turn_start[num] == False:
        turn_time[num] = rospy.get_time()
        turn_start[num] = True

    ori = current_robots.orientation[num]
    if ori in range(-5.0,5.0):
        v1 = [0,-1]
    elif ori in range(-175.0,-180.0) or ori in range (175.0,180.0):
        v1 = [0,1]
    elif ori in range(85.0,95.0):
        v1 = [-1,0]
    elif ori in range(-85.0,-95.0):
        v1 = [1,0]

    v2 = [a.x-b.x, a.y-b.y]

    theta = m.acos(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))

    if theta == 90.0:
        if v2[0] < 0:
            turn_left = True
        else:
            turn_left = False
    elif theta == 0.0:
        if v2[1] < 0:
            flip = True
        else:
            flip = False

    if turn_left:
        x = turn(0.0, velocity, 1.0)
    else:
        x = turn(velocity, 0.0, 1.0)
    if flip:
        x = turn(0.0, velocity, 2.0)

    if x:
        turn_start[num] = False
        wheel_vel[num][0], wheel_vel[num][1] = velocity, velocity
    
def shape_formation(num, msg_buff):
    global current_robots, goals, wheel_vel

    if moving[num] == True:
        return 0

    rgoal = Point()
    wpx, wpy = current_robots.wp[num].x, current_robots.wp[num].y
    surroundings = [Point(), Point(), Point(), Point()]
    surroundings[0].x, surroundings[0].y = wpx-1, wpy
    surroundings[1].x, surroundings[1].y = wpx+1, wpy
    surroundings[2].x, surroundings[2].y = wpx, wpy+1
    surroundings[3].x, surroundings[3].y = wpx, wpy-1
    wait_flag = False
    
    for i in surroundings:
        if reducedist2goal(current_robots.wp[num],i,current_robots.T[num]) == True:
            current_robots.next_step[num] = i
            break

    if msg_buff:
        min_hop_msg = findMinMsg(msg_buff)
        current_robots.hop[num] = 1 + current_robots.hop[min_hop_msg]
        current_robots.q_u[num] = current_robots.q_u[min_hop_msg]
        
        for i in surroundings:
            if  goalPoint(i):
                if goalAssigned(msg_buff, i) == False:
                    current_robots.q_u = i
                    current_robots.hop = 0
                    break

        # goal_manager  
        for i in msg_buff:
            if current_robots.T[i] == current_robots.T[num] and current_robots.index[i] > current_robots.index[num]:
                if random() > 0.1:
                    current_robots.T = current_robots.q_u
                else:
                    r = randint(0,NOR)
                    rgoal.x , rgoal.y = goals[r][0]  ,goals[r][1] 
                    current_robots.T[num] =  rgoal
                    
            if reducedist2goal(current_robots.wp[num],current_robots.T[i],current_robots.T[num]):
                temp = current_robots.T[num]
                current_robots.T[num] = current_robots.T[i]
                current_robots.T[i] = temp
            else:
                if random() > 0.6:
                    temp = current_robots.T[num]
                    current_robots.T[num] = current_robots.T[i]
                    current_robots.T[i] = temp

            # obstacle detection
            if current_robots.wp[i] == current_robots.next_step[num]:
                wait_flag == True
                wheel_vel[num][0] = 0.0
                wheel_vel[num][1] = 0.0
            if current_robots.next_step[i] == current_robots.next_step[num] and current_robots.index[i] > current_robots.index[num]:
                wait_flag == True
                wheel_vel[num][0] = 0.0
                wheel_vel[num][1] = 0.0
                
    if wait_flag == False:
        current_robots.wp[num] = current_robots.next_step[num]
        moving[num] = True
        moveRobot(current_robots.wp[num], current_robots.next_step[num])
        


rospy.init_node("shape_formation")
# handshake with robot name in parameter server, and get model urdf

robot_name = rospy.get_param("/morph_sim/robot_name")

if (robot_name != 'morph_bot'):
    rospy.logerr("wrong robot according to parameter server")
    sys.exit() # return when wrong robot manager is called


# check if gazebo is up and running by check service "/gazebo/set_joint_properties"
rospy.wait_for_service("/gazebo/set_joint_properties")
rospy.loginfo("service is ready")


morph_bot_broadcast = rospy.Publisher("/morph_sim/two_wheel_robot", BroadCast, queue_size=10)
rospy.Subscriber("/morph_sim/two_wheel_robot", BroadCast, twoWheelRobotCallback, queue_size=10) 

half_sec = rospy.Duration(0.5)
if not current_robots:
    while not current_robots:
        rospy.loginfo('waiting for publisher')
        rospy.sleep(half_sec)

rospy.loginfo('robots data recieved')

global robot_quantity
robot_quantity = len(current_robots.index)
global wheel_vel
wheel_vel = np.zeros((robot_quantity, 2), dtype=float)
turn_time = np.zeros((robot_quantity), dtype=float)
turn_start = np.zeros((robot_quantity), dtype=bool)

set_joint_properties_client = rospy.ServiceProxy("/gazebo/set_joint_properties", SetJointProperties)
set_joint_properties_srv_msg = SetJointPropertiesRequest()
set_joint_properties_srv_msg.ode_joint_config.fmax = np.zeros([1,1], dtype=float)
set_joint_properties_srv_msg.ode_joint_config.vel = np.zeros([1,1], dtype=float)
set_joint_properties_srv_msg.ode_joint_config.fmax[0] = 100.0;
 
goals,r,c = goal_points() # gets all goal points and image size
NOR = len(goals[0])       # total number of robots
goal, cgoal, wp, next_step = Point(), Point(), Point(), Point() # initializing variables

current_robots.hop = list(current_robots.hop)
for i in range(robot_quantity):
    r_goal, r_cgoal = randint(0,NOR-1), randint(0,NOR-1)          # generates random numbers
    goal.x , goal.y = goals[0][r_goal]/20.0  ,goals[1][r_goal]/20.0     # assigns random goal
    cgoal.x , cgoal.y = goals[0][r_cgoal]/20.0  ,goals[1][r_cgoal]/20.0 # assigns random goal
    wp.x, wp.y = int(current_robots.x[i]), int(current_robots.y[i])
    next_step.x, next_step.y = int(current_robots.x[i]), int(current_robots.y[i])
    current_robots.T.append(goal)
    current_robots.q_u.append(cgoal)
    current_robots.wp.append(wp)
    current_robots.next_step.append(next_step)
    current_robots.hop.append(100000)

#last_check = rospy.get_time() # current time
#dt = 2/NOR                    # time interval
#msg_buff = []                 # message buffer
#loop_hz = rospy.Rate(NOR)

while not rospy.is_shutdown():
    msg_buff = []
    # send service request of wheel velocities
    for i in range(robot_quantity):
        # for j in range(robot_quantity):
        #     if i == j: continue
        #     if m.dist([current_robots.x[i], current_robots.y[i]],
        #         [current_robots.x[j], current_robots.y[j]] ) < 0.6:
        #         msg_buff.append(current_robots.index[j])

        # shape_formation(i, msg_buff)
        # msg_buff = []
        if turn_start[i] == False:
            turn_time[i] = rospy.get_time()
            turn_start[i] = True

        rospy.loginfo(current_robots.orientation[0])
        if rospy.get_time() - turn_time[i] > 10.0:
            wheel_vel[i][0], wheel_vel[i][1] = 0.0, 0.0
        else:
             wheel_vel[i][0], wheel_vel[i][1] = 0.0, velocity
        # left wheel
        set_joint_properties_srv_msg.joint_name = "morph_bot_" + str(current_robots.index[i]) + "::left_motor"
        set_joint_properties_srv_msg.ode_joint_config.vel[0] = wheel_vel[i][0]
        set_joint_properties_response = set_joint_properties_client(set_joint_properties_srv_msg)
        if (set_joint_properties_response):
            if not (set_joint_properties_response.success):
                # possibly the robot not found
                rospy.logwarn("the robot model not found when set left wheel vel")
        else:
            rospy.logerr("fail to connect with gazebo server when set left wheel vel")
        # right wheel
        set_joint_properties_srv_msg.joint_name = "morph_bot_" + str(current_robots.index[i]) + "::right_motor"
        set_joint_properties_srv_msg.ode_joint_config.vel[0] = wheel_vel[i][1]
        set_joint_properties_response = set_joint_properties_client(set_joint_properties_srv_msg)
        if (set_joint_properties_response):
            if not (set_joint_properties_response.success):
                # possibly the robot not found
                rospy.logwarn("the robot model not found when set right wheel vel")
        else:
            rospy.logerr("fail to connect with gazebo server when set right wheel vel")
    
    #morph_bot_broadcast.publish(current_robots)
    #loop_hz.sleep()


