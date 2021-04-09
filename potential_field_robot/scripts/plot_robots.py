#!/usr/bin/python


import numpy as np
import rospy
from morph_msg.msg import RobotState
from matplotlib import pyplot as plt
from im2goal import goal_points
plt.style.use('seaborn-whitegrid')

goals,r,c = goal_points()
NOR = len(goals[0]) # number of robots

pos = [ []*NOR for _ in range(NOR)]

def recieve_state(rcvd_msg):
	global posx, posy
	name = rcvd_msg.robot_name.data
	robot_no = int(name[6]) # gets robot number
	pos[robot_no] = [rcvd_msg.pos.x, rcvd_msg.pos.y]
	#posy[robot_no].append(rcvd_msg.pos.y)

rospy.init_node("plotter")
rospy.Subscriber("robot_state", RobotState, recieve_state, queue_size=100)



while not rospy.is_shutdown():
	plt.clf()
	plt.axis([0, r, 0, c])
	#print(pos)
	for i in range(NOR):
		if pos[i]:
			plt.plot(pos[i][0],pos[i][1],marker ='o')
		else:
			continue
	plt.pause(0.01)
	plt.draw()
	#plt.show()
