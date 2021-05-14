#!/usr/bin/python

import sys
import rospy
import math as m
import numpy as np
from im2goal import goal_points

from random import random
from random import uniform
from random import randint

from std_msgs.msg import String
from morph_msg.msg import BroadCast
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion, Pose, Point

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest
from swarm_robot_srv.srv import two_wheel_robot_update, two_wheel_robot_updateResponse

global current_robots, goals, NOR, gf

current_robots = BroadCast()
robot_position_updated = False
#current_robots.hop = list(current_robots.hop)

gf = 5.0 # goal division factor
goals,r,c = goal_points() # gets all goal points and image size
NOR = len(goals[0])       # total number of robots
goals = goals/gf


def quaternion_to_angle(input_quaternion):
    # this assume the x and y element of the quaternion is close to zero
    rad = m.atan(input_quaternion.z/input_quaternion.w) * 2
    if rad < 0:
        rad += 2*m.pi
    deg = rad * (180/m.pi)
    return deg

def random_quaternion():
    # alpha is the rotation angle along z axis
    half_alpha = (random() % 360 - 180)/2.0 * m.pi/180.0
    output_quaternion = Quaternion()
    output_quaternion.x = 0
    output_quaternion.y = 0
    #output_quaternion.z = m.sin(half_alpha)
    #output_quaternion.w = m.cos(half_alpha)
    output_quaternion.z = 0
    output_quaternion.w = 0
    return output_quaternion

def random_position(half_range):
    position_2d = np.array([0.0,0.0])
    # generate the pseudorandom float number between [-half_range, half_range]
    position_2d[0] = uniform(-half_range, half_range)
    position_2d[1] = uniform(-half_range, half_range)
    return position_2d

def position_availibility(position_2d):
    current_robot_quantity = len(current_robots.index)
    for i in range(current_robot_quantity-1):
        dist = m.sqrt(pow(position_2d[0] - current_robots.x[i], 2) + pow(position_2d[1] - current_robots.y[i], 2))
        if (dist < 0.0465): 
            return False
    return True

def twoWheelRobotUpdateCallback(request):
    response = two_wheel_robot_updateResponse()
    response.response_code = two_wheel_robot_update._response_class.SUCCESS
    if (request.update_code <= two_wheel_robot_update._request_class.CODE_DELETE):
        container_index = current_robots.index
        delete_robot_quantity = abs(request.update_code)
        current_robot_quantity = len(container_index)

        if (delete_robot_quantity > current_robot_quantity):
            rospy.logerr("requested deletion quantity exceeds existed robots")
            response.response_code = two_wheel_robot_update._response_class.DELETE_FAIL_EXCEED_QUANTITY
            return response
        else:
            delete_model = DeleteModelRequest()
            for i in range(delete_robot_quantity):
                container_index_index = random() % len(container_index)
                delete_index = container_index[container_index_index]
                container_index.pop(container_index_index)
                delete_model.model_name = "morph_bot_" + str(delete_index)
                delete_response = delete_model_client(delete_model)
                if (delete_response):
                    if (delete_response.success):
                        rospy.loginfo("morph_bot_{0} has been deleted".format(delete_index))
                    else:
                        rospy.loginfo("morph_bot_{0} deletion failed".format(delete_index))
                        response.response_code = two_wheel_robot_update._response_class.FAIL_OTHER_REASONS
                else:
                    rospy.logerr("fail to connect with gazebo server")
                    response.response_code = two_wheel_robot_update._response_class.DELETE_FAIL_NO_RESPONSE
                    return response
            return response
 
    elif (request.update_code == two_wheel_robot_update._request_class.CODE_DELETE_ALL):
        delete_model = DeleteModelRequest()
        container_index = current_robots.index[:]
        delete_robot_quantity = len(container_index)
        for i in range(delete_robot_quantity):
            delete_index = container_index[i]
            delete_model.model_name = "morph_bot_" + str(delete_index)
            delete_response = delete_model_client(delete_model)
            if (delete_response):
               if (delete_response.success):
                   rospy.loginfo("morph_bot_{0} has been deleted".format(delete_index))
               else:
                   rospy.loginfo("morph_bot_{0} deletion failed".format(delete_index))
                   response.response_code = two_wheel_robot_update._response_class.FAIL_OTHER_REASONS
            else:
               rospy.logerr("fail to connect with gazebo server")
               response.response_code = two_wheel_robot_update._response_class.DELETE_FAIL_NO_RESPONSE
               return response
        return response
    
    elif (request.update_code >= two_wheel_robot_update._request_class.CODE_ADD):
        add_model = SpawnModelRequest()
        new_position = np.array([0,0])
        if (request.add_mode == two_wheel_robot_update._request_class.ADD_MODE_RANDOM):
            new_position = random_position(request.half_range)
            position_generating_count = 1
            while (position_availibility(new_position) == False and position_generating_count < 100):
                new_position = random_position(request.half_range)
                position_generating_count += 1

            if (position_generating_count == 100):
                rospy.logerr("add robot at random fail because range is too crowded")
                response.response_code = two_wheel_robot_update._response_class.ADD_FAIL_TOO_CROWDED
                return response
        
        elif (request.add_mode == two_wheel_robot_update._request_class.ADD_MODE_SPECIFIED):
            new_position = request.position_2d
            if not position_availibility(new_position): 
                rospy.logerr("add robot at specified position fail because it is occupied")
                response.response_code = two_wheel_robot_update._response_class.ADD_FAIL_OCCUPIED
                return response
            
        new_model_name = ''
        #rospy.loginfo(current_robots.index)
        if (len(current_robots.index) == 0): 
            new_model_name = "morph_bot_0"
        else:
            new_model_name = "morph_bot_" + str(current_robots.index[-1] + 1)

        add_model.model_name = new_model_name
        add_model.model_xml = two_wheel_robot_urdf
        add_model.robot_namespace = new_model_name
        add_model.initial_pose.position.x = new_position[0]
        add_model.initial_pose.position.y = new_position[1]
        add_model.initial_pose.position.z = 0.0
        add_model.initial_pose.orientation = random_quaternion()
        add_model_response = add_model_client(add_model)
        if (add_model_response): 
            if (add_model_response.success):
                rospy.loginfo("{0} has been spawned".format(new_model_name))
            else:
                rospy.loginfo("{0} fail to be spawned".format(new_model_name))
                response.response_code = two_wheel_robot_update._response_class.FAIL_OTHER_REASONS
        else:
            rospy.logerr("fail to connect with gazebo server")
            response.response_code = two_wheel_robot_update._response_class.ADD_FAIL_NO_RESPONSE
            return response
        return response
    
# callback for getting robot positions
# also check and update if there is any addition or deletion of robots
def modelStatesCallback(current_model_states):
    global robot_position_updated
    model_quantity = len(current_model_states.name)
    # the index of robots in the container
    container_index = current_robots.index[:]
    container_size = len(container_index)
    for i in range(model_quantity): 
        # check if it is a two wheel robot
        # there is a underscore between the name and the index
        found = current_model_states.name[i].find("morph_bot")
        if (found != -1):
            # 10 = 9 + 1, 15 is the length of "morph_bot"
            index_str = current_model_states.name[i][10:]
            index_parsed = int(index_str);
            # search in the container
            parsed_index_found = False
            for j in range(container_size):
                # the size of container_index may change for each i
                if (index_parsed == container_index[j]):
                    # update the 2D position of two wheel robots
                    current_robots.x[j] = round(current_model_states.pose[i].position.x,3)
                    current_robots.y[j] = round(current_model_states.pose[i].position.y,3)
                    current_robots.orientation[j] = round(quaternion_to_angle(current_model_states.pose[i].orientation),3)
                    # mark as -1, meaning position is updated
                    # will check later if there is any element not marked, i.e., not in gazebo
                    container_index[j] = -1
                    parsed_index_found = True
                    break
                
            if not parsed_index_found: 
                # parsed index not found in the container, ADDITION found!
                # update a new robot in the container
                current_robots.index.append(index_parsed)
                current_robots.x.append(round(current_model_states.pose[i].position.x,3)) # x coordinate
                current_robots.y.append(round(current_model_states.pose[i].position.y,3)) # y coordinate
                current_robots.orientation.append(quaternion_to_angle(current_model_states.pose[i].orientation)) # orientation
                current_robots.left_wheel_vel.append(0.0)
                current_robots.right_wheel_vel.append(0.0)
                goal, cgoal, wp, next_step = Point(), Point(), Point(), Point() # initializing variables
                # r_goal, r_cgoal = randint(0,NOR-1), randint(0,NOR-1)          # generates random numbers
                # goal.x , goal.y = goals[0][r_goal]  ,goals[1][r_goal]   # assigns random goal
                # cgoal.x , cgoal.y = goals[0][r_cgoal] ,goals[1][r_cgoal] # assigns random goal
                goal.x , goal.y = goals[0][i-1]  ,goals[1][i-1] # unique goals assigned
                cgoal.x , cgoal.y = goals[0][i-1],goals[1][i-1] # unique candidate goals assigned
                wp.x, wp.y = current_model_states.pose[i].position.x, current_model_states.pose[i].position.y
                next_step.x, next_step.y = wp.x, wp.y # waypoint = next_step
                current_robots.T.append(goal) # target goal
                current_robots.q_u.append(cgoal) # candidate goal
                current_robots.wp.append(wp) # waypoint
                current_robots.next_step.append(next_step) # next step
                current_robots.hop.append(100000) # hop count
                rospy.loginfo("robot addition detected: morph_bot_%s" % index_str)
    
    # update the container if there is any deletion in gazebo
    for i in range(container_size):
        if (container_index[i] != -1):
            # not marked with -1, there is a deletion
            current_robots_index_size = len(current_robots.index)
            for i in range(current_robots_index_size):
                # the index should be found before this loop ends
                if (container_index[i] == current_robots.index[j]):
                    # pop the robot
                    current_robots.index.pop(j)
                    current_robots.x.pop(j)
                    current_robots.y.pop(j)
                    current_robots.orientation.pop(j)
                    current_robots.left_wheel_vel.pop(j)
                    current_robots.right_wheel_vel.pop(j)
                    rospy.loginfo("robot deletion detected: morph_bot_%s" % container_index[i])
                    break       
    # reset robot position updated flag
    robot_position_updated = True


rospy.init_node("morph_bot_manager")
# handshake with robot name in parameter server, and get model urdf

robot_name = rospy.get_param("/morph_sim/robot_name")
global two_wheel_robot_urdf
two_wheel_robot_urdf = rospy.get_param("/morph_sim/two_wheel_robot_urdf")

if (not (robot_name and two_wheel_robot_urdf)):
    rospy.logerr("simulation environmnet(parameters) is not set")
    sys.exit()  # return when parameter server is not good

if (robot_name != 'morph_bot'):
    rospy.logerr("wrong robot according to parameter server")
    sys.exit() # return when wrong robot manager is called


# check if gazebo is up and running by check service "/gazebo/set_physics_properties"
# this service seems like the last service hosted by gazebo
rospy.wait_for_service("/gazebo/set_physics_properties")
rospy.loginfo("gazebo is ready")

# instantiate a publisher for the maintained information of two wheel robot
morph_bot_pub = rospy.Publisher("/morph_sim/morph_bot", BroadCast, queue_size=10)

# instantiate a subscriber for "/gazebo/model_states"
rospy.Subscriber("/gazebo/model_states", ModelStates, modelStatesCallback)
# this topic publish at 1000hz rate

# instantiate a service client to get wheel velocities
joint_properties_client = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
joint_properties_srv_msg = GetJointPropertiesRequest()

# instantiate a service client for "/gazebo/spawn_urdf_model"
global add_model_client
add_model_client = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
# instantiate a service client for "/gazebo/delete_model"
global delete_model_client
delete_model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

# instantiate a service server to modify the robots in gazebo
# add or delete robot models in gazebo
two_wheel_robot_service = rospy.Service("/morph_sim/two_wheel_robot_update", two_wheel_robot_update, twoWheelRobotUpdateCallback)

#loop_hz = rospy.Rate(10)
# publish loop
while not rospy.is_shutdown():
    # get the wheel speed for the maintained robots
    current_robot_quantity = len(current_robots.index)
    if (robot_position_updated):
        for i in range(current_robot_quantity):
            # get left wheel velocity
            joint_properties_srv_msg.joint_name = "morph_bot_" + str(current_robots.index[i]) + "::left_motor"
            joint_properties_response = joint_properties_client(joint_properties_srv_msg)
            # if call service not successful, it will leave the joint velocity unchanged
            if (joint_properties_response):
                if not joint_properties_response.success:
                    # joint not found, there is possible robot deletion the container doesn't realize
                    rospy.logwarn("there is possible robot deletion not updated")
                else: 
                    # update the left wheel velocity
                    current_robots.left_wheel_vel[i] = round(joint_properties_response.rate[0],3)
            else: 
                rospy.logerr("fail to connect with gazebo server")
                # do not return here
            
            # get right wheel velocity
            joint_properties_srv_msg.joint_name = "morph_bot_" + str(current_robots.index[i]) + "::right_motor"
            joint_properties_response = joint_properties_client(joint_properties_srv_msg)
            # if call service not successful, it will leave the joint velocity unchanged
            if (joint_properties_response):
                if not joint_properties_response.success: 
                    # joint not found, there is possible robot deletion the container doesn't realize
                    rospy.logwarn("there is possible robot deletion not updated")  
                else:
                    # update the right wheel velocity
                    current_robots.right_wheel_vel[i] = round(joint_properties_response.rate[0],3)
            else:
                rospy.logerr("fail to connect with gazebo server")
        # publish the two wheel robot information container
        morph_bot_pub.publish(current_robots)
        # reset the position update flag
        robot_position_updated = False
    # update global variables
    #loop_hz.sleep()


    

