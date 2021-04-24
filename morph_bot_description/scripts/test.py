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
from gazebo_msgs.srv import SetJointProperties, SetJointPropertiesRequest

current_robots = BroadCast() # robot's local info
        
def twoWheelRobotCallback(rcvd_msg):
    global msg_buff
    current_robots.index = rcvd_msg.index
    current_robots.x = rcvd_msg.x
    current_robots.y = rcvd_msg.y
    current_robots.orientation = rcvd_msg.orientation
    current_robots.left_wheel_vel = rcvd_msg.left_wheel_vel
    current_robots.right_wheel_vel = rcvd_msg.right_wheel_vel

rospy.init_node('test')
# check if gazebo is up and running by check service "/gazebo/set_joint_properties"
rospy.wait_for_service("/gazebo/set_joint_properties")
rospy.loginfo("service is ready")

rospy.Subscriber("/morph_sim/two_wheel_robot", BroadCast, twoWheelRobotCallback, queue_size=10) 

set_joint_properties_client = rospy.ServiceProxy("/gazebo/set_joint_properties", SetJointProperties)
set_joint_properties_srv_msg = SetJointPropertiesRequest()
#set_joint_properties_srv_msg.ode_joint_config.fmax[0] = 1.0;
rospy.loginfo('bruh')
loop_hz = rospy.Rate(64)

while not rospy.is_shutdown():
    robot_quantity = len(current_robots.index)
    # send service request of wheel velocities
    for i in range(robot_quantity):
        # left wheel
        set_joint_properties_srv_msg.joint_name = "morph_bot_" + str(current_robots.index[i]) + "::left_motor"
        set_joint_properties_srv_msg.ode_joint_config.vel[0] = 0.05
        set_joint_properties_response = set_joint_properties_client(set_joint_properties_srv_msg)
        rospy.loginfo(set_joint_properties_response.success)
        if (set_joint_properties_response):
            if not (set_joint_properties_response.success):
                # possibly the robot not found
                rospy.logwarn("the robot model not found when set left wheel vel")
            
        
        else:
            rospy.logerr("fail to connect with gazebo server when set left wheel vel")
        # right wheel
        set_joint_properties_srv_msg.joint_name = "morph_bot_" + str(current_robots.index[i]) + "::right_motor"
        set_joint_properties_srv_msg.ode_joint_config.vel[0] = 0.05
        set_joint_properties_response = set_joint_properties_client(set_joint_properties_srv_msg)
        rospy.loginfo(set_joint_properties_response.success)
        if (set_joint_properties_response):
            if not (set_joint_properties_response.success):
                # possibly the robot not found
                rospy.logwarn("the robot model not found when set right wheel vel")
        else:
            rospy.logerr("fail to connect with gazebo server when set right wheel vel")
    
    loop_hz.sleep()
    rospy.spin()  # let the global variables update

