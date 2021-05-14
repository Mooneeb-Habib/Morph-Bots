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

global current_robots, velocity, lin_time, gl, fwd_vel, gf
gl = 0.2
gf = 5.0
current_robots = None
lin_vel = 0.12
wheel_rad = 0.015
velocity = 3.454
fwd_vel = 4.0163*2
lin_time = gl/lin_vel
initialized = False
count = 0

goals,r,c = goal_points() # gets all goal points and image size
NOR = len(goals[0])       # total number of robots
goals = goals/gf
rospy.loginfo(goals)
rospy.sleep(1.0)

def twoWheelRobotCallback(rcvd_msg):
    global current_robots, initialized
    if not current_robots:
        current_robots = BroadCast()
    # last_check = rospy.get_time()
    if initialized == True:
        current_robots.index = rcvd_msg.index
        current_robots.x = rcvd_msg.x
        current_robots.y = rcvd_msg.y
        current_robots.orientation = rcvd_msg.orientation
        current_robots.left_wheel_vel = rcvd_msg.left_wheel_vel
        current_robots.right_wheel_vel = rcvd_msg.left_wheel_vel
    else:
        current_robots = rcvd_msg
        current_robots.hop = list(rcvd_msg.hop)
        initialized = True

# def PathPlannerCallback(msg):
#     global current_robots
#     if not current_robots:
#         current_robots = BroadCast()
#     # last_check = rospy.get_time()
#     current_robots.wp = msg.wp
#     current_robots.next_step = msg.next_step
#     current_robots.T = msg.T
#     current_robots.q_u = msg.q_u
#     current_robots.hop = list(msg.hop)

def check_equal(a,b):
    a.x = round(a.x, 2)
    a.y = round(a.y, 2)
    b.x = round(b.x, 2)
    b.y = round(b.y, 2)

    if a.x == b.x and a.y == b.y:
        return True
    else:
        return False

def reducedist2goal(pos, point, goal):
    curr_dist = abs(pos.x - goal.x) + abs(pos.y - goal.y)
    next_dist = abs(point.x - goal.x) + abs(point.y - goal.y)
    if next_dist < curr_dist:
        return True
    return False

def swapGoalDistReduce(apos, bpos, agoal, bgoal):
    curr_dist_a = abs(apos.x - agoal.x) + abs(apos.y - agoal.y)
    curr_dist_b = abs(bpos.x - bgoal.x) + abs(bpos.y - bgoal.y)
    swap_dist_a = abs(apos.x - bgoal.x) + abs(apos.y - bgoal.y)
    swap_dist_b = abs(bpos.x - agoal.x) + abs(bpos.y - agoal.y)
    overall_curr_dist = curr_dist_a + curr_dist_b
    overall_swap_dist = swap_dist_a + swap_dist_b

    return(overall_swap_dist < overall_curr_dist)

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
        if x[0] == round(i.x, 2) and x[1] == round(i.y, 2) :
            return True
    return False 
    
def goalAssigned(msg_buff, goal):
    for i in msg_buff:
        if check_equal(current_robots.T[i], goal):
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
    if rospy.get_time() - move_time[num] < (lin_time):
        wheel_vel[num][0], wheel_vel[num][1] = fwd_vel, fwd_vel
        return False
    else:
        wheel_vel[num][0], wheel_vel[num][1] = 0.0, 0.0
        return True

def waiting(num):
    if rospy.get_time() - wait_time[num] < (lin_time + 10.0):
        wheel_vel[num][0], wheel_vel[num][1] = 0.0, 0.0
    else:
        wait_flag[num] = False

def moveRobot(a, b, num):
    global velocity, moving, linear_move, last_check

    rx, ry = current_robots.x[num], current_robots.y[num]

    if turn_start[num] == False and linear_move[num] == False:
        turn_time[num] = rospy.get_time()
        turn_start[num] = True

        ori = 90 * (round(current_robots.orientation[num]/90))
        theta = abs((m.atan2(b.y-a.y, b.x-a.x)*180.0)/m.pi)
        dphi = theta - ori
        #rospy.loginfo("dphi for {0} @ ({2},{3}) and ({4},{5}) = {1}".format(num, dphi, a.x, a.y, b.x, b.y))

        if dphi == 90.0 or dphi == -270.0:
            turn_choice[num] = 1
        elif dphi == -90.0 or dphi == 270.0:
            turn_choice[num] = 2
        elif abs(dphi) == 180.0:
            turn_choice[num] = 3
        elif abs(dphi) == 360.0 or abs(dphi) == 0.0:
            turn_choice[num] = 4            

    if turn_choice[num] == 1:
        x = turn(0.0, velocity, 1.0, num)
    elif turn_choice[num] == 2:
        x = turn(velocity, 0.0, 1.0, num)
    elif turn_choice[num] == 3:
        x = turn(-velocity, velocity, 0.98, num)
    elif turn_choice[num] == 4:
        x = turn(0.0, 0.0, 1.0, num)

    if x and linear_move[num] == False:
        turn_start[num] = False
        linear_move[num] = True
        move_time[num] = rospy.get_time()

    if linear_move[num] == True:
        lm = move(num)
        if lm:
            moving[num] = False
            linear_move[num] = False
            last_check[num] = rospy.get_time()
                
                
def shape_formation(num, msg_buff):
    global current_robots, goals, wheel_vel, moving
    #rospy.loginfo(num)
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

    for s in surroundings:
        if reducedist2goal(current_robots.wp[num],s,current_robots.T[num]) == True:
            current_robots.next_step[num] = s
            break
    rospy.loginfo("--------------------------------")
    rospy.loginfo("waypoint for {2} : ({0},{1})".format(current_robots.wp[num].x,current_robots.wp[num].y,num))
    # rospy.loginfo("potential next step: {0}".format(i))
    rospy.loginfo("goal for {2} : ({0},{1})".format(current_robots.T[num].x,current_robots.T[num].y,num))
    rospy.loginfo("next step for {2} : ({0},{1})".format(current_robots.next_step[num].x,current_robots.next_step[num].y,num))
    rospy.loginfo("--------------------------------")

    if msg_buff:
        min_hop_msg = findMinMsg(msg_buff)
        current_robots.hop[num] = 1 + current_robots.hop[min_hop_msg]
        current_robots.q_u[num] = current_robots.q_u[min_hop_msg]
        
        for i in surroundings:
            if  goalPoint(i):
                if goalAssigned(msg_buff, i) == False:
                    current_robots.q_u[num] = i
                    current_robots.hop[num] = 0
                    break

    #     # goal_manager  
        for i in msg_buff:
            if check_equal(current_robots.T[i], current_robots.T[num]) and current_robots.index[i] > current_robots.index[num]:
                if random() > 0.1:
                    current_robots.T[num] = current_robots.q_u[num]
                    rospy.loginfo('goal swap with candidate goal')
                    last_check[num] = rospy.get_time()

                else:
                    r = randint(0,NOR-1)
                    rgoal.x , rgoal.y = goals[0][r] ,goals[1][r]
                    current_robots.T[num] =  rgoal
                    rospy.loginfo('random goal swap if goals equal')
                    last_check[num] = rospy.get_time()
                    
            if swapGoalDistReduce(current_robots.wp[num], current_robots.wp[i], current_robots.T[num],current_robots.T[i]):
                temp = current_robots.T[num]
                current_robots.T[num] = current_robots.T[i]
                current_robots.T[i] = temp
                rospy.loginfo('goal swap if overall reduced distance')
                last_check[num] = rospy.get_time()
            else:
                if random() > 0.9:
                    temp = current_robots.T[num]
                    current_robots.T[num] = current_robots.T[i]
                    current_robots.T[i] = temp
                    rospy.loginfo('goal swap anyways randomly')
                    last_check[num] = rospy.get_time()

            # obstacle detection
            if check_equal(current_robots.wp[i], current_robots.next_step[num]):
                wait_flag[num] = True
                wait_time = rospy.get_time()
                rospy.loginfo('obstacle detected on next waypoint')
            if check_equal(current_robots.next_step[i], current_robots.next_step[num]):# and current_robots.index[i] > current_robots.index[num]:
                wait_flag[num] = True
                wait_time = rospy.get_time()
                rospy.loginfo('obstacle incoming on next step')

    if wait_flag[num] == False:
        check[num] = False
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


# morph_bot_broadcast = rospy.Publisher("/morph_sim/path_planner", BroadCast, queue_size=10)
rospy.Subscriber("/morph_sim/morph_bot", BroadCast, twoWheelRobotCallback, queue_size=10) 
# rospy.Subscriber("/morph_sim/path_planner", BroadCast, PathPlannerCallback, queue_size=10)

half_sec = rospy.Duration(0.5)
if not current_robots:
    while not current_robots:
        rospy.loginfo('waiting for publisher')
        rospy.sleep(half_sec)

rospy.loginfo('robots data recieved')

global robot_quantity
robot_quantity = len(current_robots.index)

global wheel_vel, turn_time, turn_start, turn_choice, moving, wait_time, move_time 
global linear_move, last_check, check, wait_flag, reach, flip
wheel_vel = np.zeros((robot_quantity, 2), dtype=float)
last_check = np.zeros((robot_quantity), dtype=float)
turn_time = np.zeros((robot_quantity), dtype=float)
wait_time = np.zeros((robot_quantity), dtype=float)
move_time = np.zeros((robot_quantity), dtype=float)
turn_start = np.zeros((robot_quantity), dtype=bool)
turn_choice = np.zeros((robot_quantity), dtype=int)
flip = np.zeros((robot_quantity), dtype=bool)
moving = np.zeros((robot_quantity), dtype=bool)
linear_move = np.zeros((robot_quantity), dtype=bool)
check = np.zeros((robot_quantity), dtype=bool)
wait_flag = np.zeros((robot_quantity), dtype=bool)
reach = np.zeros((robot_quantity), dtype=bool)

set_joint_properties_client = rospy.ServiceProxy("/gazebo/set_joint_properties", SetJointProperties)
set_joint_properties_srv_msg = SetJointPropertiesRequest()
set_joint_properties_srv_msg.ode_joint_config.fmax = np.zeros([1,1], dtype=float)
set_joint_properties_srv_msg.ode_joint_config.vel = np.zeros([1,1], dtype=float)
set_joint_properties_srv_msg.ode_joint_config.fmax[0] = 1000.0;


# loop_hz = rospy.Rate(100)

while not rospy.is_shutdown():
    msg_buff = []
    # send service request of wheel velocities
    for i in range(robot_quantity):
        #rospy.loginfo(moving[i])
        if moving[i] == False and wait_flag[i] == False:
            current_robots.wp[i] = current_robots.next_step[i]
            for j in range(robot_quantity):
                if i == j: continue
                distance = m.sqrt(((current_robots.wp[j].x-current_robots.wp[i].x)**2)+((current_robots.wp[j].y-current_robots.wp[i].y)**2))
                if distance < 1.0:
                    msg_buff.append(current_robots.index[j])
            shape_formation(i, msg_buff)
            msg_buff = []
            # rospy.loginfo("actual coordinates for {2}: ({0},{1})".format(round(current_robots.x[i],3),round(current_robots.y[i],3), i))

        elif moving[i] == True and ((rospy.get_time() - last_check[i]) > 2.0):
            if check_equal(current_robots.wp[i], current_robots.T[i]):
                if reach[i] == False:
                    count += 1
                    reach[i] = True
                    rospy.loginfo('goal reached at {0}'.format(current_robots.T[i]))
                wheel_vel[i][0], wheel_vel[i][1] = 0.0, 0.0
            else:
                moveRobot(current_robots.wp[i], current_robots.next_step[i], i)
        elif wait_flag[i] == True:
            waiting(i)
            rospy.loginfo("..robot {0} is waiting..".format(i))
        
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

    # if robot_quantity:
    #     if count > robot_quantity:
    #         rospy.loginfo('goals reached!')
    #         sys.exit()
    # morph_bot_broadcast.publish(current_robots)
    # loop_hz.sleep()


