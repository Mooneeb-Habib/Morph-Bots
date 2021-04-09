#!/usr/bin/python

import rospy
import math
import sys
import numpy as np
from random import random 
from std_msgs.msg import String
from morph_msg.msg import RobotState
from im2goal import goal_points



zeta = 0.1; # attractive field strength
eta = 0.1; # repulsive field strength
rho0 = 1;
radius = 0.2; # radius of the robot
index = 1

def recieve_state(rcvd_msg):
	global obs
	name = rcvd_msg.robot_name.data
	print(name)
	rospy.loginfo("I heard %s" % name )
	obs = rcvd_msg
	

def potential_field():
	global index, zeta, eta , rho0, radius, robot, goal
	if (obs.robot_name.data != rospy.get_name()):
		print('Start: ', robot)
		if (index == 1):
			Gradient_x = zeta*(robot[0]-goal[0]) # gradient in x direction
			Gradient_y = zeta*(robot[1]-goal[1]) # gradient in y direction
			xObs = obs.pos.x # x - coordinate of other robot
			yObs = obs.pos.y # y - coordinate of other robot
			#xObs = obs[0] - robot[0] # x - coordinate of other robot
			#yObs = obs[1] - robot[1] # y - coordinate of other robot
			deg = math.atan2(yObs - robot[1], xObs - robot[0]) # angle between obs and robot
			xObs1 = xObs - robot[0]
			yObs1 = yObs - robot[1]
			di = np.linalg.norm([xObs1, yObs1]) - radius; # distance to obs center
			xCl = robot[0] + di * math.cos(deg) # x - coordinate of closest point
			yCl = robot[1] + di * math.sin(deg) # y - coordinate of closest point
			# if the distance from obs center is less than rho0 then there is repulsive force applied by other robot
			if (di <= rho0):
				# finds the repulsive gradient
				Gradient_x = Gradient_x + eta*((rho0**-1)-(di**-1))*((di**-1))**3*(robot[0]-xCl) # gradient in x direction
				Gradient_y = Gradient_y + eta*((rho0**-1)-(di**-1))*((di**-1))**3*(robot[1]-yCl) # gradient in y direction
			Force = [-Gradient_x, -Gradient_y] # repulsive force vector
			robot = np.add(robot, Force) # force applied to current position
			print('End: ', robot)
			msg.pos.x = robot[0]
			msg.pos.y = robot[1]
			msg.pos.z = 0
			#if robot is in accuracy range of goal the motion ends
			if (abs(robot[0] - goal[0]) < 0.1 and abs(robot[1] - goal[1]) < 0.1):
				index = 2
	    

rospy.init_node("robot_state_n")
pub = rospy.Publisher("robot_state", RobotState, queue_size=10000)
 
goals,r,c = goal_points() # gets all goal points and image size
NOR = len(goals[0])
name = rospy.get_name() # gets node name
robot_no = int(name[6]) # gets robot number
goal = [goals[0][robot_no], goals[1][robot_no]] # gets goal
robot = [random()*r, random()*c] # assigns random position

msg = RobotState() # robot's local info
obs = RobotState() # other robot's info
msg.robot_name.data = rospy.get_name()
msg.pos.x = robot[0]
msg.pos.y = robot[1]
msg.pos.z = 0

rospy.Subscriber("robot_state", RobotState, recieve_state, queue_size=10000)
loop_hz = rospy.Rate(NOR)

while not rospy.is_shutdown():
	potential_field()
	pub.publish(msg)
	rospy.loginfo(msg)
	loop_hz.sleep()

