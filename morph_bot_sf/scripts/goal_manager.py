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
from goal_extractor.gpExtractor import csv2gp

global current_robots, gf, mb_range
gf = 10.0 # goal factor
current_robots = None # initializing array
mb_range = 0.5

goals,mid,r,c = goal_points() # gets all goal points and image size
NOR = len(goals[0])       # total number of robots
goals = goals/gf


def PathPlannerCallback(msg):
    global current_robots
    if not current_robots:
        current_robots = BroadCast()
    current_robots = msg
    current_robots.hop = list(current_robots.hop)
    current_robots.last_check = list(current_robots.last_check)

# to check if two Points are equal
def check_equal(a,b):
    a.x = round(a.x, 2)
    a.y = round(a.y, 2)
    b.x = round(b.x, 2)
    b.y = round(b.y, 2)

    if a.x == b.x and a.y == b.y:
        return True
    else:
        return False

# to check if goal swap reduces overall Manhattan distance travelled for two robots.
def swapGoalDistReduce(apos, bpos, agoal, bgoal):
    curr_dist_a = abs(apos.x - agoal.x) + abs(apos.y - agoal.y)
    curr_dist_b = abs(bpos.x - bgoal.x) + abs(bpos.y - bgoal.y)
    swap_dist_a = abs(apos.x - bgoal.x) + abs(apos.y - bgoal.y)
    swap_dist_b = abs(bpos.x - agoal.x) + abs(bpos.y - agoal.y)
    overall_curr_dist = curr_dist_a + curr_dist_b
    overall_swap_dist = swap_dist_a + swap_dist_b

    return(round(overall_swap_dist,3) < round(overall_curr_dist,3))
        
                
def goal_manager(num, msg_buff):
    global current_robots, goals
    rgoal = Point()

    if msg_buff: # if neighbours in range

        # goal_manager  
        for i in msg_buff: # in neighbours
            # if neigbour goal is equal to own goal and its priority is greater
            if check_equal(current_robots.T[i], current_robots.T[num]) and current_robots.index[i] > current_robots.index[num]:
                if random() > 0.1:
                    current_robots.T[num] = current_robots.q_u[num] # swap target goal with candidate goal
                    #rospy.loginfo('goal swap with candidate goal')
                    current_robots.last_check = list(current_robots.last_check)
                    current_robots.last_check[num] = rospy.get_time() # reset time check
                else:
                    # assigns random goal for duplicate goal assignments
                    r = randint(0,NOR-1) 
                    rgoal.x , rgoal.y = goals[0][r] ,goals[1][r] 
                    current_robots.T[num] =  rgoal
                    #rospy.loginfo('random goal swap if goals equal')
                    current_robots.last_check = list(current_robots.last_check)
                    current_robots.last_check[num] = rospy.get_time() # reset time check

            # swaps goal with neighbour if overall Manhattan distance travelled is reduced
            if swapGoalDistReduce(current_robots.wp[num], current_robots.wp[i], current_robots.T[num],current_robots.T[i]):
                #if random() < 0.7:
                temp = current_robots.T[num]
                current_robots.T[num] = current_robots.T[i]
                current_robots.T[i] = temp
                #rospy.loginfo('goal swap if overall reduced distance')
                current_robots.last_check = list(current_robots.last_check)
                current_robots.last_check[num] = rospy.get_time() # reset time check
            else:
                if random() > 0.99: # randomly will swap goal anyways to reduce greediness of algorithm
                    temp = current_robots.T[num]
                    current_robots.T[num] = current_robots.T[i]
                    current_robots.T[i] = temp
                    #rospy.loginfo('goal swap anyways randomly')
                    current_robots.last_check = list(current_robots.last_check)
                    current_robots.last_check[num] = rospy.get_time() # reset time check

       
        

# initializing node
rospy.init_node("goal_manager")

goal_broadcast = rospy.Publisher("/morph_sim/goal_manager", BroadCast, queue_size=10)
rospy.Subscriber("/morph_sim/path_planner", BroadCast, PathPlannerCallback, queue_size=100)

# waits for robot data to be recieved
half_sec = rospy.Duration(0.5)
if not current_robots:
    while not current_robots:
        rospy.loginfo('waiting for publisher')
        rospy.sleep(half_sec)

rospy.loginfo('robots data recieved')

global robot_quantity
robot_quantity = len(current_robots.index) # number of robots in the simulation

loop_hz = rospy.Rate(NOR)

while not rospy.is_shutdown():
    msg_buff = [] # message buffer for each robot

    for i in range(robot_quantity): # run for each robot
        # get messages of other robots in range
        for j in range(robot_quantity):
            if i == j: continue
            distance = m.sqrt(((current_robots.wp[j].x-current_robots.wp[i].x)**2) + \
                       ((current_robots.wp[j].y-current_robots.wp[i].y)**2))
            if distance < mb_range: # if distance is in range
                msg_buff.append(current_robots.index[j]) # add index to message buffer
        goal_manager(i, msg_buff)
        msg_buff = []
    goal_broadcast.publish(current_robots)
    loop_hz.sleep()


