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
from gazebo_msgs.srv import SetJointProperties, SetJointPropertiesRequest

global current_robots, velocity, lin_time, gl, fwd_vel, gf, mb_range
gl = 0.1 # grid length
gf = 10.0 # goal factor
current_robots = None # initializing array
lin_vel = 0.12 # linear velocity
wheel_rad = 0.015 # wheel radius
velocity = 3.454 # turning velocity
fwd_vel = 4.0163*2 # forward movement velocity
lin_time = gl/lin_vel #  forward movement time
initialized = False # boolean for initialzing array
count = 0
mb_range = 0.5

goals,mid,r,c = goal_points() # gets all goal points and image size
NOR = len(goals[0])       # total number of robots
goals = goals/gf

def twoWheelRobotCallback(rcvd_msg):
    global current_robots, initialized
    if not current_robots:
        current_robots = BroadCast()
    # last_check = rospy.get_time()
    if initialized == True:
        # only update these values to prevent overwriting
        current_robots.index = rcvd_msg.index
        current_robots.x = rcvd_msg.x
        current_robots.y = rcvd_msg.y
        current_robots.orientation = rcvd_msg.orientation
        current_robots.left_wheel_vel = rcvd_msg.left_wheel_vel
        current_robots.right_wheel_vel = rcvd_msg.left_wheel_vel
    else:
        # takes all values to intialize algorithm
        current_robots = rcvd_msg
        # ROS deserializes arrays as tuples hence it has to be converted back
        current_robots.hop = list(rcvd_msg.hop)
        current_robots.last_check = list(rcvd_msg.last_check)
        initialized = True # array has been initialized

def GoalManagerCallback(msg):
    global current_robots
    current_robots.T = msg.T
    current_robots.q_u = msg.q_u
    current_robots.last_check = msg.last_check
    current_robots.last_check = list(msg.last_check)

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

# to check if robots next step reduces Manhattan distance to goal
def reducedist2goal(pos, point, goal):
    curr_dist = abs(pos.x - goal.x) + abs(pos.y - goal.y)
    next_dist = abs(point.x - goal.x) + abs(point.y - goal.y)
    if next_dist < curr_dist:
        return True
    return False

# to check if goal swap reduces overall Manhattan distance travelled for two robots.
def swapGoalDistReduce(apos, bpos, agoal, bgoal):
    curr_dist_a = abs(apos.x - agoal.x) + abs(apos.y - agoal.y)
    curr_dist_b = abs(bpos.x - bgoal.x) + abs(bpos.y - bgoal.y)
    swap_dist_a = abs(apos.x - bgoal.x) + abs(apos.y - bgoal.y)
    swap_dist_b = abs(bpos.x - agoal.x) + abs(bpos.y - agoal.y)
    overall_curr_dist = curr_dist_a + curr_dist_b
    overall_swap_dist = swap_dist_a + swap_dist_b

    return(overall_swap_dist < overall_curr_dist)

# to find the minimum hop count value
def findMinMsg(msg_buff):
    min_hop = 100000
    min_msg = 0
    for i in msg_buff:
        if current_robots.hop[i] <= min_hop:
            min_hop = current_robots.hop[i]
            min_msg = i
    rospy.loginfo("hop for {0} = {1}".format(min_msg,min_hop))
    return min_msg

# to check if a waypoint is a goal
def goalPoint(i):
    global goals
    for x in range(NOR):
        if goals[0][x] == round(i.x, 2) and goals[1][x] == round(i.y, 2) :
            return True
    return False 

# to check if a goal is assigned to another robot
def goalAssigned(msg_buff, goal):
    for i in msg_buff:
        if check_equal(current_robots.T[i], goal):
            return True
    return False 

# implements robot turning
def turn(vr, vl, t, num):
    if rospy.get_time() - turn_time[num] < t:
        wheel_vel[num][0], wheel_vel[num][1] = vr, vl # keep turning for specified time.
        return False
    else:
        wheel_vel[num][0], wheel_vel[num][1] = 0.0, 0.0 # if turn time complete robot stopped
        return True

# implements robots forward movement
def move(num, yes_move):
    if rospy.get_time() - move_time[num] < (lin_time):
        if yes_move:
            wheel_vel[num][0], wheel_vel[num][1] = fwd_vel, fwd_vel  # keep moving for specified time.
        else:
            wheel_vel[num][0], wheel_vel[num][1] = 0.0, 0.0
        return False
    else:
        wheel_vel[num][0], wheel_vel[num][1] = 0.0, 0.0 # if moving time complete robot stopped
        return True

# implements waiting time if wait_flag == True
def waiting(num):
    if rospy.get_time() - wait_time[num] < (lin_time):
        wheel_vel[num][0], wheel_vel[num][1] = 0.0, 0.0
    else:
        wait_flag[num] = False

# implements complete robot movement
def moveRobot(a, b, num):
    global velocity, moving, linear_move, last_check

    #rx, ry = current_robots.x[num], current_robots.y[num]

    if turn_start[num] == False and linear_move[num] == False:
        turn_time[num] = rospy.get_time()
        turn_start[num] = True

        ori = 90 * (round(current_robots.orientation[num]/90)) # current orientation in 90 degree steps
        # ori_actual = round(current_robots.orientation[num])
        theta = abs((m.atan2(b.y-a.y, b.x-a.x)*180.0)/m.pi) # angle to which robot has to face
        dphi = theta - ori # change in angle required to achieve theta
        #rospy.loginfo("dphi for {0} @ ({2},{3}) and ({4},{5}) = {1}".format(num, dphi, a.x, a.y, b.x, b.y))

        if dphi == 90.0 or dphi == -270.0:
            turn_choice[num] = 1 # left turn
        elif dphi == -90.0 or dphi == 270.0:
            turn_choice[num] = 2 # right turn
        elif abs(dphi) == 180.0:
            turn_choice[num] = 3 # turn around to face back
        elif abs(dphi) == 360.0 or abs(dphi) == 0.0:
            turn_choice[num] = 4 # no turn

        # dphi_actual = theta - ori_actual   
        # t[num] = (1.0/90.0)* abs(dphi_actual)

    if turn_choice[num] == 1:
        x = turn(-velocity, velocity, 0.515, num) # applies velocity on right wheel
    elif turn_choice[num] == 2:
        x = turn(velocity, -velocity, 0.515, num) # applies velocity on left wheel
    elif turn_choice[num] == 3:
        x = turn(-velocity, velocity, 0.515*2, num) # applies velocity on right wheel for double time
    elif turn_choice[num] == 4:
        x = turn(0.0, 0.0, 2.0, num) # waits in the time other robots are turning

    if x and linear_move[num] == False: # turn completed, linear movement start
        turn_start[num] = False
        linear_move[num] = True
        move_time[num] = rospy.get_time() # movement timer started

    if linear_move[num] == True:
        if not check_equal(a,b):
            lm = move(num, True) # moves the robot over specified grid length
        else:
            lm = move(num, False) 
        if lm:
            moving[num] = False # robot stopped
            linear_move[num] = False # movement finished
            current_robots.last_check = list(current_robots.last_check)
            current_robots.last_check[num] = rospy.get_time() # time check renewed
                
                
def shape_formation(num, msg_buff):
    global current_robots, goals, wheel_vel, moving
    #rospy.loginfo(num)
    rgoal = Point()
    wait_flag[num] = False

    if check[num] == False:
        check[num] = True
        current_robots.last_check = list(current_robots.last_check)
        current_robots.last_check[num] = rospy.get_time()

    wpx, wpy = current_robots.wp[num].x, current_robots.wp[num].y # taking waypoint coordinates
    surroundings = [Point(), Point(), Point(), Point()] # intiializing surroundings
    surroundings[0].x, surroundings[0].y = wpx-gl, wpy # x - 1 neighbour 
    surroundings[1].x, surroundings[1].y = wpx+gl, wpy # x + 1 neighbour
    surroundings[2].x, surroundings[2].y = wpx, wpy+gl # y + 1 neighbour
    surroundings[3].x, surroundings[3].y = wpx, wpy-gl # y - 1 neighbour

    # check which next_step will reduce distance to goal
    for s in surroundings:
        if reducedist2goal(current_robots.wp[num],s,current_robots.T[num]) == True:
            current_robots.next_step[num] = s
            break

    # printing for debugging
    # rospy.loginfo("--------------------------------")
    # rospy.loginfo("waypoint for {2} : ({0},{1})".format(current_robots.wp[num].x,current_robots.wp[num].y,num))
    # # rospy.loginfo("potential next step: {0}".format(i))
    # rospy.loginfo("goal for {2} : ({0},{1})".format(current_robots.T[5].x,current_robots.T[5].y,5))
    # rospy.loginfo("next step for {2} : ({0},{1})".format(current_robots.next_step[num].x,current_robots.next_step[num].y,num))
    # rospy.loginfo("--------------------------------")

    if msg_buff and ((rospy.get_time() - current_robots.last_check[num]) > 2.0/rate): # if neighbours in range
        min_hop_msg = findMinMsg(msg_buff) # finds robot with minimum hop count
        current_robots.hop[num] = 1 + current_robots.hop[min_hop_msg] # own hop count becomes min + 1
        current_robots.q_u[num] = current_robots.q_u[min_hop_msg] # takes candidate goal of min_hop robot
        
        for i in surroundings:
            if goalPoint(i): # if any surrounding is a goal point 
                if goalAssigned(msg_buff, i) == False: # and it is not assigned to any other robot
                    current_robots.q_u[num] = i # make it candidate goal
                    current_robots.hop[num] = 0 # reset hop count to zero
                    break
        for i in msg_buff:
            # if current next step is another's waypoint don't move forward
            if check_equal(current_robots.wp[i], current_robots.next_step[num]):
                wait_flag[num] = True # wait flag set to true
                #wait_time[num] = rospy.get_time() # wait timer started
                #rospy.loginfo('obstacle detected on next waypoint')
            # if current next step is another's next step and its priority is greater don't move forward
            if check_equal(current_robots.next_step[i], current_robots.next_step[num]) and current_robots.index[i] > current_robots.index[num]:
                wait_flag[num] = True # wait flag set to true
                #wait_time[num] = rospy.get_time() # wait timer started
                #rospy.loginfo('obstacle incoming on next step')

    if wait_flag[num] == False and ((rospy.get_time() - current_robots.last_check[num]) > 2.0/rate): # if wait flag is false robot will move
        # moving_coords[num][0] = current_robots.wp[num].x
        # moving_coords[num][1] = current_robots.wp[num].y
        # moving_coords[num][2] = current_robots.next_step[num].x
        # moving_coords[num][3] = current_robots.next_step[num].y
        current_robots.wp[num] = current_robots.next_step[num] # update waypoint
        check[num] = False # now time check will be reset in next iteration
        # moving[num] = True # now moveRobot function can run

       
        

# initializing node
rospy.init_node("shape_formation")
# handshake with robot name in parameter server
robot_name = rospy.get_param("/morph_sim/robot_name")

if (robot_name != 'morph_bot'):
    rospy.logerr("wrong robot according to parameter server")
    sys.exit() # return when wrong robot manager is called


# check if gazebo is up and running by check service "/gazebo/set_joint_properties"
rospy.wait_for_service("/gazebo/set_joint_properties")
rospy.loginfo("service is ready")


morph_bot_broadcast = rospy.Publisher("/morph_sim/path_planner", BroadCast, queue_size=10)
# rospy.Subscriber("/morph_sim/path_planner", BroadCast, PathPlannerCallback, queue_size=10)

# subscriber listens to morph bot manager 
rospy.Subscriber("/morph_sim/morph_bot", BroadCast, twoWheelRobotCallback, queue_size=10)
# subscriber listens to morph bot manager 
rospy.Subscriber("/morph_sim/goal_manager", BroadCast, GoalManagerCallback, queue_size=10) 

# waits for robot data to be recieved
half_sec = rospy.Duration(0.5)
if not current_robots:
    while not current_robots:
        rospy.loginfo('waiting for publisher')
        rospy.sleep(half_sec)

rospy.loginfo('robots data recieved')

global robot_quantity
robot_quantity = len(current_robots.index) # number of robots in the simulation

global wheel_vel, last_check, turn_time, wait_time, move_time, turn_choice, moving_coords
global turn_start, moving, linear_move, check, wait_flag, reach

wheel_vel = np.zeros((robot_quantity, 2), dtype=float) # wheel velocities
moving_coords = np.zeros((robot_quantity, 4), dtype=float) # coordinates for moving the robots
last_check = np.zeros((robot_quantity), dtype=float)   # timer check for shape formation
turn_time = np.zeros((robot_quantity), dtype=float)    # turn timer
wait_time = np.zeros((robot_quantity), dtype=float)    # waiting timer
move_time = np.zeros((robot_quantity), dtype=float)    # linear movement timer  
turn_choice = np.zeros((robot_quantity), dtype=int)    # chooses which direction to turn

turn_start = np.zeros((robot_quantity), dtype=bool)    # checks whether turning has started
moving = np.zeros((robot_quantity), dtype=bool)        # checks whether robot is moving
linear_move = np.zeros((robot_quantity), dtype=bool)   # checks whether robot is moving forward
check = np.zeros((robot_quantity), dtype=bool)         # checks whether last_check timer has been initialized
wait_flag = np.zeros((robot_quantity), dtype=bool)     # checks whether robot should move or stay at current waypoint
reach = np.zeros((robot_quantity), dtype=bool)         # checks if robot has reached its goal

# set up service client for calling gazebo service for setting joint velocities
set_joint_properties_client = rospy.ServiceProxy("/gazebo/set_joint_properties", SetJointProperties)
set_joint_properties_srv_msg = SetJointPropertiesRequest() # creating service request
set_joint_properties_srv_msg.ode_joint_config.fmax = np.zeros([1,1], dtype=float) # max force
set_joint_properties_srv_msg.ode_joint_config.vel = np.zeros([1,1], dtype=float) # joint velocity
set_joint_properties_srv_msg.ode_joint_config.fmax[0] = 1000.0; # setting max force

global rate
rate = NOR
loop_hz = rospy.Rate(rate)

while not rospy.is_shutdown():
    msg_buff = [] # message buffer for each robot

    for i in range(robot_quantity): # run for each robot
        # if robot is stationary and not waiting
        if moving[i] == False: 
            # get messages of other robots in range
            for j in range(robot_quantity):
                if i == j: continue
                distance = m.sqrt(((current_robots.wp[j].x-current_robots.wp[i].x)**2) + \
                           ((current_robots.wp[j].y-current_robots.wp[i].y)**2))
                if distance < mb_range: # if distance is in range
                    msg_buff.append(current_robots.index[j]) # add index to message buffer

            shape_formation(i, msg_buff) # call shape formation path planner
            msg_buff = [] # clear msg_buffer for next robot
            # rospy.loginfo("actual coordinates for {2}: ({0},{1})".format(round(current_robots.x[i],3),\
            # round(current_robots.y[i],3), i))
            if current_robots.hop[i] == int(robot_quantity*1.5):
                rospy.loginfo(current_robots.hop[i])
                rospy.loginfo('ALL GOALS REACHED') 
                sys.exit()
            # if check_equal(current_robots.wp[i], current_robots.T[i]):
            #     if reach[i] == False: # if it has not reached
            #         reach[i] = True # set reached check True
            #         rospy.loginfo('goal reached at {0}'.format(current_robots.T[i]))
        # if robot is supposed to move
        # if moving[i] == True:
        #     # check if robot has reached the goal
        #     if check_equal(current_robots.wp[i], current_robots.T[i]):
        #         if reach[i] == False: # if it has not reached
        #             count += 1
        #             reach[i] = True # set reached check True
        #             rospy.loginfo('goal reached at {0}'.format(current_robots.T[i]))
        #         wheel_vel[i][0], wheel_vel[i][1] = 0.0, 0.0 # stop robot
        #     else:
        #         # call robot movement function
        #         a, b = Point(), Point()
        #         a.x, a.y = moving_coords[i][0], moving_coords[i][1]
        #         b.x, b.y = moving_coords[i][2], moving_coords[i][3]
        #         moveRobot(a,b,i)  

        # # send service request of wheel velocities
        # set_joint_properties_srv_msg.joint_name = "morph_bot_" + str(current_robots.index[i]) + "::left_motor"
        # set_joint_properties_srv_msg.ode_joint_config.vel[0] = wheel_vel[i][0]
        # set_joint_properties_response = set_joint_properties_client(set_joint_properties_srv_msg)
        # if (set_joint_properties_response):
        #     if not (set_joint_properties_response.success):
        #         # possibly the robot not found
        #         rospy.logwarn("the robot model not found when set left wheel vel")
        # else:
        #     rospy.logerr("fail to connect with gazebo server when set left wheel vel")
        # # right wheel
        # set_joint_properties_srv_msg.joint_name = "morph_bot_" + str(current_robots.index[i]) + "::right_motor"
        # set_joint_properties_srv_msg.ode_joint_config.vel[0] = wheel_vel[i][1]
        # set_joint_properties_response = set_joint_properties_client(set_joint_properties_srv_msg)
        # if (set_joint_properties_response):
        #     if not (set_joint_properties_response.success):
        #         # possibly the robot not found
        #         rospy.logwarn("the robot model not found when set right wheel vel")
        # else:
        #     rospy.logerr("fail to connect with gazebo server when set right wheel vel")

    for x in range(robot_quantity):
        for y in range(robot_quantity):
            if x == y: continue
            if check_equal(current_robots.wp[x], current_robots.wp[y]):
                rospy.logerr('robot {0} and {1} go to horny jail'.format(x,y))
    # if robot_quantity:
    #     if count > robot_quantity:
    #         rospy.loginfo('goals reached!')
    #         sys.exit()
    morph_bot_broadcast.publish(current_robots)
    loop_hz.sleep()


