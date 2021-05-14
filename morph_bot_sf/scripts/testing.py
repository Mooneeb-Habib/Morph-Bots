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

global current_robots
current_robots = None
def twoWheelRobotCallback(rcvd_msg):
    global current_robots, initialized
    if not current_robots:
        current_robots = BroadCast()
    current_robots = rcvd_msg

def reducedist2goal(pos, point, goal):
    curr_dist = abs(pos.x - goal.x) + abs(pos.y - goal.y)
    next_dist = abs(point.x - goal.x) + abs(point.y - goal.y)
    if next_dist < curr_dist:
        return True
    return False

def check_equal(a,b):
	a.x = round(a.x, 2)
	a.y = round(a.y, 2)
	b.x = round(b.x, 2)
	b.y = round(b.y, 2)

	if a.x == b.x and a.y == b.y:
		return True
	else:
		return False

def turn(vr, vl, t):
    if rospy.get_time() - turn_time < t:
        wheel_velx, wheel_vely = vr, vl
        return False
    else:
        wheel_velx, wheel_vely = 0.0, 0.0
        return True

# gl = 0.2
# wp = Point()
# wp.x, wp.y = 0.0, 0.0
# next_step = Point()
# T = Point()
# T.x, T.y = 1.0, 0.4
# wpx, wpy = wp.x, wp.y

rospy.init_node("testing")

global wheel_velx, wheel_vely
wheel_velx, wheel_vely = 0.0, 3.454

rospy.Subscriber("/morph_sim/morph_bot", BroadCast, twoWheelRobotCallback, queue_size=10)

half_sec = rospy.Duration(0.5)
if not current_robots:
    while not current_robots:
        rospy.loginfo('waiting for publisher')
        rospy.sleep(half_sec)

rospy.loginfo('robots data recieved')

set_joint_properties_client = rospy.ServiceProxy("/gazebo/set_joint_properties", SetJointProperties)
set_joint_properties_srv_msg = SetJointPropertiesRequest()
set_joint_properties_srv_msg.ode_joint_config.fmax = np.zeros([1,1], dtype=float)
set_joint_properties_srv_msg.ode_joint_config.vel = np.zeros([1,1], dtype=float)
set_joint_properties_srv_msg.ode_joint_config.fmax[0] = 1000.0;


# while not rospy.is_shutdown():
# 	wpx, wpy = wp.x, wp.y 
# 	surroundings = [Point(), Point(), Point(), Point()]
# 	surroundings[0].x, surroundings[0].y = wpx-gl, wpy
# 	surroundings[1].x, surroundings[1].y = wpx+gl, wpy
# 	surroundings[2].x, surroundings[2].y = wpx, wpy+gl
# 	surroundings[3].x, surroundings[3].y = wpx, wpy-gl

# 	for i in surroundings:
# 	    if reducedist2goal(wp,i,T) == True:
# 	        next_step = i
# 	        break

# 	wp = next_step
# 	rospy.loginfo("waypoint = ({0},{1})".format(wp.x,wp.y))
# 	if check_equal(wp,T):
# 		rospy.loginfo('goal reached')
# 		sys.exit()
timer = rospy.get_time()
while not rospy.is_shutdown():
	wheel_velx, wheel_vely = -3.454, 3.454
	#rospy.loginfo(current_robots.orientation[0])
	if current_robots.orientation[0] > 350:
		x = rospy.get_time() - timer
		rospy.loginfo(x)
	set_joint_properties_srv_msg.joint_name = "morph_bot_" + str(0) + "::left_motor"
	set_joint_properties_srv_msg.ode_joint_config.vel[0] = wheel_velx
	set_joint_properties_response = set_joint_properties_client(set_joint_properties_srv_msg)
	if (set_joint_properties_response):
	    if not (set_joint_properties_response.success):
	        # possibly the robot not found
	        rospy.logwarn("the robot model not found when set left wheel vel")
	else:
	    rospy.logerr("fail to connect with gazebo server when set left wheel vel")
	# right wheel
	set_joint_properties_srv_msg.joint_name = "morph_bot_" + str(0) + "::right_motor"
	set_joint_properties_srv_msg.ode_joint_config.vel[0] = wheel_vely
	set_joint_properties_response = set_joint_properties_client(set_joint_properties_srv_msg)
	if (set_joint_properties_response):
	    if not (set_joint_properties_response.success):
	        # possibly the robot not found
	        rospy.logwarn("the robot model not found when set right wheel vel")
	else:
	    rospy.logerr("fail to connect with gazebo server when set right wheel vel")