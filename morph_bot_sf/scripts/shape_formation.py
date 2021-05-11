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
    if not current_robots:
        current_robots = BroadCast()
        current_robots.hop = list(current_robots.hop)
    # last_check = rospy.get_time()
    current_robots = rcvd_msg
    current_robots.hop = list(current_robots.hop)
        
def reducedist2goal(pos, point, goal):
    curr_dist = abs(pos.x - goal.x) + abs(pos.y - goal.y)
    next_dist = abs(point.x - goal.x) + abs(point.y - goal.y)
    if next_dist < curr_dist:
        return True
    return False

def findMinMsg(msg_buff):
    min_hop = 100000
    min_msg = 0
    for i in msg_buff:
        if current_robots.hop[i] <= min_hop:
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

def turn(vr, vl, t, num):
    if rospy.get_time() - turn_time[num] < t:
        wheel_vel[num][0], wheel_vel[num][1] = vr, vl
        return False
    else:
        wheel_vel[num][0], wheel_vel[num][1] = 0.0, 0.0
        return True

def move(num):
    if rospy.get_time() - turn_time[num] < 0.0833:
        wheel_vel[num][0], wheel_vel[num][1] = velocity, velocity
        return False
    else:
        wheel_vel[num][0], wheel_vel[num][1] = 0.0, 0.0
        return True

def moveRobot(a, b, num):
    global velocity

    rx, ry = current_robots.x[num], current_robots.y[num]

    if turn_start[num] == False and linear_move[num] == False:
        turn_time[num] = rospy.get_time()
        turn_start[num] = True
        ori = current_robots.orientation[num]

        theta = (m.atan2(a.x-b.x, a.y-b.y)*180)/m.pi
        dphi = 90.0 * round((theta - ori)/90.0)

        if dphi == 90.0:
            turn_left[num] = True
        elif dphi == -90.0:
            turn_left[num] = False
        elif dphi == 180.0:
            flip[num] = True


    if turn_left[num]:
        x = turn(0.0, velocity, 1.0, num)
    else:
        x = turn(velocity, 0.0, 1.0, num)
    if flip[num]:
        x = turn(0.0, velocity, 2.0, num)

    if x and linear_move[num] == False:
        turn_start[num] = False
        linear_move[num] = True
        turn_time[num] = rospy.get_time()

    if linear_move[num] == True:
        lm = move(num)
        if lm:
            moving[num] = False
            linear_move[num] = False
            last_check[num] = rospy.get_time()
                
                

    
def shape_formation(num, msg_buff):
    global current_robots, goals, wheel_vel
    wait_flag = False
    if moving[num] == False:
        rgoal = Point()
        if check[num] == False:
            check[num] = True
            last_check[num] = rospy.get_time()
        wpx, wpy = current_robots.wp[num].x, current_robots.wp[num].y
        surroundings = [Point(), Point(), Point(), Point()]
        surroundings[0].x, surroundings[0].y = wpx-gl, wpy
        surroundings[1].x, surroundings[1].y = wpx+gl, wpy
        surroundings[2].x, surroundings[2].y = wpx, wpy+gl
        surroundings[3].x, surroundings[3].y = wpx, wpy-gl
        
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
                        current_robots.T[num] = current_robots.q_u[num]
                    else:
                        r = randint(0,NOR-1)
                        rgoal.x , rgoal.y = goals[0][r]  ,goals[1][r] 
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
                    wait_flag = True
                    wheel_vel[num][0] = 0.0
                    wheel_vel[num][1] = 0.0
                if current_robots.next_step[i] == current_robots.next_step[num] and current_robots.index[i] > current_robots.index[num]:
                    wait_flag = True
                    wheel_vel[num][0] = 0.0
                    wheel_vel[num][1] = 0.0

    if wait_flag == False and ((rospy.get_time() - last_check[num]) > 2.0):
        check[num] = False
        moveRobot(current_robots.wp[num], current_robots.next_step[num], num)
        current_robots.wp[num] = current_robots.next_step[num]
        moving[num] = True
       
        


rospy.init_node("shape_formation")
# handshake with robot name in parameter server, and get model urdf

robot_name = rospy.get_param("/morph_sim/robot_name")

if (robot_name != 'morph_bot'):
    rospy.logerr("wrong robot according to parameter server")
    sys.exit() # return when wrong robot manager is called


# check if gazebo is up and running by check service "/gazebo/set_joint_properties"
rospy.wait_for_service("/gazebo/set_joint_properties")
rospy.loginfo("service is ready")


morph_bot_broadcast = rospy.Publisher("/morph_sim/morph_bot", BroadCast, queue_size=10)
rospy.Subscriber("/morph_sim/morph_bot", BroadCast, twoWheelRobotCallback, queue_size=10) 

half_sec = rospy.Duration(0.5)
if not current_robots:
    while not current_robots:
        rospy.loginfo('waiting for publisher')
        rospy.sleep(half_sec)

rospy.loginfo('robots data recieved')

global robot_quantity, gl
gl = 0.05
robot_quantity = len(current_robots.index)

global wheel_vel, turn_time, turn_start, turn_left, flip, moving, linear_move, last_check, check
wheel_vel = np.zeros((robot_quantity, 2), dtype=float)
last_check = np.zeros((robot_quantity), dtype=float)
turn_time = np.zeros((robot_quantity), dtype=float)
turn_start = np.zeros((robot_quantity), dtype=bool)
turn_left = np.zeros((robot_quantity), dtype=bool)
flip = np.zeros((robot_quantity), dtype=bool)
moving = np.zeros((robot_quantity), dtype=bool)
linear_move = np.zeros((robot_quantity), dtype=bool)
check = np.zeros((robot_quantity), dtype=bool)

set_joint_properties_client = rospy.ServiceProxy("/gazebo/set_joint_properties", SetJointProperties)
set_joint_properties_srv_msg = SetJointPropertiesRequest()
set_joint_properties_srv_msg.ode_joint_config.fmax = np.zeros([1,1], dtype=float)
set_joint_properties_srv_msg.ode_joint_config.vel = np.zeros([1,1], dtype=float)
set_joint_properties_srv_msg.ode_joint_config.fmax[0] = 100.0;

goals,r,c = goal_points() # gets all goal points and image size
NOR = len(goals[0])       # total number of robots

loop_hz = rospy.Rate(20)

while not rospy.is_shutdown():
    msg_buff = []
    # send service request of wheel velocities
    for i in range(robot_quantity):
        for j in range(robot_quantity):
            if i == j: continue
            distance = m.sqrt(((current_robots.x[j]-current_robots.x[i])**2)+((current_robots.y[j]-current_robots.y[i])**2))
            if distance < 0.3:
                msg_buff.append(current_robots.index[j])

        shape_formation(i, msg_buff)
        msg_buff = []

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
    
    morph_bot_broadcast.publish(current_robots)
    loop_hz.sleep()


