#!/usr/bin/python


import numpy as np
import rospy
from morph_msg.msg import BroadCast
from geometry_msgs.msg import Point
import matplotlib as mpl
from matplotlib import pyplot as plt
from im2goal import goal_points
plt.style.use('seaborn-whitegrid')
mpl.rc('xtick', labelsize=4) 
mpl.rc('ytick', labelsize=4) 
global robots, robot_quantity
goals,mid,r,c = goal_points()
r /= 10; c /= 10
NOR = len(goals[0]) # number of robots
robots = None
robot_quantity = NOR


def recieve_state(current_robots):
	global robots, robot_quantity
	# robot_quantity = len(current_robots.index)
	# pos = [ []*robot_quantity for _ in range(robot_quantity)]
	# for i in range(robot_quantity):
	# 	pos[i] = [current_robots.wp[i].x, current_robots.wp[i].y]
	robots = current_robots
	robot_quantity = len(current_robots.index)

rospy.init_node("plotter")
rospy.Subscriber("/morph_sim/path_planner", BroadCast, recieve_state, queue_size=100)

# waits for robot data to be recieved
half_sec = rospy.Duration(0.5)
if not robots:
    while not robots:
        rospy.loginfo('waiting for publisher')
        rospy.sleep(half_sec)

rospy.loginfo('robots data recieved')


while not rospy.is_shutdown():
	pos = robots.wp
	plt.xticks(np.arange(0,r, step = 0.1))
	plt.yticks(np.arange(0,c, step = 0.1))
	plt.axis([0, r, 0, c])
	#print(pos)
	for i in range(robot_quantity):
		plt.plot(pos[i].x, pos[i].y,marker ='${0}$'.format(i),markersize = 4)
	plt.pause(0.000000001)
	plt.draw()
	plt.clf()
	#plt.show()