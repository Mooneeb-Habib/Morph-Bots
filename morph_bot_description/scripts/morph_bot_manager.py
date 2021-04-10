#!/usr/bin/python

import rospy
import sys
from random import random
from random import uniform
import numpy as np
import math as m
from swarm_robot_msg.msg import two_wheel_robot
from swarm_robot_srv.srv import two_wheel_robot_update
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetJointProperties
from geometry_msgs.msg import Quaternion

safe_dist = 0.0465

global current_robots, robot_position_updated
current_robots = two_wheel_robot()
robot_position_updated = False

def quaternion_to_angle(input_quaternion):
    # this assume the x and y element of the quaternion is close to zero
    return m.atan(input_quaternion.z/input_quaternion.w) * 2

def random_quaternion():
    # alpha is the rotation angle along z axis
    half_alpha = (random() % 360 - 180)/2.0 * m.pi/180.0
    output_quaternion = Quaternion()
    output_quaternion.x = 0
    output_quaternion.y = 0
    output_quaternion.z = m.sin(half_alpha)
    output_quaternion.w = m.cos(half_alpha)
    return output_quaternion

def random_position(half_range):
    position_2d = np.array(2)
    # generate the pseudorandom float number between [-half_range, half_range]
    position_2d[0] = uniform(-half_range, half_range)
    position_2d[1] = uniform(-half_range, half_range)
    return position_2d

def position_availibility(position_2d):
    current_robot_quantity = len(current_robots.index)
    for i in range(current_robot_quantity):
        dist = m.sqrt(pow(position_2d[0] - current_robots.x[i], 2) + pow(position_2d[1] - current_robots.y[i], 2))
        if (dist < SAFE_DIST): return false
    return True

def twoWheelRobotUpdateCallback(request, response, add_model_client, delete_model_client, two_wheel_robot_urdf):

    if (request.update_code <= two_wheel_robot_update.CODE_DELETE):
        container_index = current_robots.index
        delete_robot_quantity = abs(request.update_code)
        current_robot_quantity = len(container_index)

        if (delete_robot_quantity > current_robot_quantity):
            rospy.logerr("requested deletion quantity exceeds existed robots")
            response.response_code = two_wheel_robot_update.DELETE_FAIL_EXCEED_QUANTITY
            return True
        else:
            delete_model = DeleteModel()
            for i in range(delete_robot_quantity):
                container_index_index = random() % len(container_index)
                delete_index = container_index[container_index_index]
                container_index.pop(container_index_index)
                delete_model.request.model_name = "morph_bot_" + str(delete_index)
                call_service = delete_model_client.call(delete_model)
                if (call_service):
                    if (delete_model.response.success):
                        rospy.loginfo("morph_bot_%s" % delete_index % "%s has been deleted")
                    else:
                        rospy.loginfo("morph_bot_%s" % delete_index % "%s deletion failed")
                        response.response_code = two_wheel_robot_update.FAIL_OTHER_REASONS
                else:
                    rospy.logerr("fail to connect with gazebo server")
                    response.response_code = two_wheel_robot_update.DELETE_FAIL_NO_RESPONSE
                    return True
            return True
 
    elif (request.update_code == two_wheel_robot_update.CODE_DELETE_ALL):
        delete_model = DeleteModel()
        container_index = current_robots.index
	delete_robot_quantity = len(container_index)
        for i in range(delete_robot_quantity):
            delete_index = container_index[i]
            delete_model.request.model_name = "morph_bot_" + str(delete_index)
            call_service = delete_model_client.call(delete_model)
            if (call_service):
               if (delete_model.response.success):
                   rospy.loginfo("morph_bot_%s" % delete_index % "%s has been deleted")
               else:
                   rospy.loginfo("morph_bot_robot_%s" % delete_index % "%s deletion failed")
                   response.response_code = two_wheel_robot_update.FAIL_OTHER_REASONS
            else:
               rospy.logerr("fail to connect with gazebo server")
               response.response_code = two_wheel_robot_update.DELETE_FAIL_NO_RESPONSE
               return True
        return True
    
    elif (request.update_code >= two_wheel_robot_update.CODE_ADD):
        add_model = SpawnModel()
        new_position = np.array(2)
        if (request.add_mode == two_wheel_robot_update.ADD_MODE_RANDOM):
            new_position = random_position(request.half_range)
            position_generating_count = 1
            while not (position_availibility(new_position) and position_generating_count < 100):
                new_position = random_position(request.half_range)
                position_generating_count += 1

            if (position_generating_count == 100):
                rospy.logerr("add robot at random fail because range is too crowded")
                response.response_code = two_wheel_robot_update.ADD_FAIL_TOO_CROWDED
                return True
        
        elif (request.add_mode == two_wheel_robot_update.ADD_MODE_SPECIFIED):
            new_position = request.position_2d
            if not position_availibility(new_position): 
                rospy.logerr("add robot at specified position fail because it is occupied")
                response.response_code = two_wheel_robot_update.ADD_FAIL_OCCUPIED
                return True
            
        new_model_name = ''
        if (len(current_robots.index) == 0): 
            new_model_name = "morph_bot_0"
        else:
            new_model_name = "morph_bot_" + str(current_robots.index[-1] + 1)
        
        add_model.request.model_name = new_model_name
        add_model.request.model_xml = two_wheel_robot_urdf
        add_model.request.robot_namespace = new_model_name
        add_model.request.initial_pose.position.x = new_position[0]
        add_model.request.initial_pose.position.y = new_position[1]
        add_model.request.initial_pose.position.z = 0.0
        add_model.request.initial_pose.orientation = random_quaternion()
        call_service = add_model_client.call(add_model)
        if (call_service): 
            if (add_model.response.success):
                rospy.loginfo(new_model_name % "%s has been spawned")
            else:
                rospy.loginfo(new_model_name % " %s fail to be spawned")
                response.response_code = two_wheel_robot_update.FAIL_OTHER_REASONS
        else:
            rospy.logerr("fail to connect with gazebo server")
            response.response_code = two_wheel_robot_update.ADD_FAIL_NO_RESPONSE
            return True
        return True
    
# callback for getting robot positions
# also check and update if there is any addition or deletion of robots
def modelStatesCallback(current_model_states):
    model_quantity = len(current_model_states.name)
    # the index of robots in the container
    container_index = current_robots.index
    container_size = len(container_index)
    for i in range(model_quantity): 
        # check if it is a two wheel robot
        # there is a underscore between the name and the index
        found = current_model_states.name[i].find("morph_bot")
        if (found != -1):
            # 16 = 15 + 1, 15 is the length of "two_wheel_robot"
            index_str = current_model_states.name[i][16]
            index_parsed = int(index_str)
            # search in the container
            parsed_index_found = false
            for j in range(container_size):
                # the size of container_index may change for each i
                if (index_parsed == container_index[j]):
                    # update the 2D position of two wheel robots
                    current_robots.x[j] = current_model_states.pose[i].position.x
                    current_robots.y[j] = current_model_states.pose[i].position.y
                    current_robots.orientation[j] = quaternion_to_angle(current_model_states.pose[i].orientation)
                    # mark as -1, meaning position is updated
                    # will check later if there is any element not marked, i.e., not in gazebo
                    container_index[j] = -1
                    parsed_index_found = True
                    break
                
            if not parsed_index_found: 
                # parsed index not found in the container, ADDITION found!
                # update a new robot in the container
                current_robots.index.append(index_parsed)
                current_robots.x.append(current_model_states.pose[i].position.x)
                current_robots.y.append(current_model_states.pose[i].position.y)
                current_robots.orientation.append(quaternion_to_angle(current_model_states.pose[i].orientation))
                current_robots.left_wheel_vel.append(0)
                current_robots.right_wheel_vel.append(0)
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
two_wheel_robot_urdf = rospy.get_param("/morph_sim/two_wheel_robot_urdf")

if (not (robot_name and two_wheel_robot_urdf)):
	rospy.logerr("simulation environmnet(parameters) is not set")
	sys.exit()  # return when parameter server is not good

rospy.loginfo(robot_name)
if (robot_name != 'morph_bot'):
	rospy.logerr("wrong robot according to parameter server")
	sys.exit() # return when wrong robot manager is called


# check if gazebo is up and running by check service "/gazebo/set_physics_properties"
# this service seems like the last service hosted by gazebo
rospy.wait_for_service("/gazebo/set_physics_properties")
rospy.loginfo("gazebo is ready")

# instantiate a publisher for the maintained information of two wheel robot
morph_bot_publisher = rospy.Publisher("/morph_sim/two_wheel_robot", two_wheel_robot, queue_size=10)

# instantiate a subscriber for "/gazebo/model_states"
model_states_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, modelStatesCallback)
# this topic publish at 1000hz rate

# instantiate a service client to get wheel velocities
joint_properties_client = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
joint_properties_srv_msg = GetJointProperties()

# instantiate a service client for "/gazebo/spawn_urdf_model"
add_model_client = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
# instantiate a service client for "/gazebo/delete_model"
delete_model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

# instantiate a service server to modify the robots in gazebo
# add or delete robot models in gazebo
two_wheel_robot_service = rospy.Service("/morph_sim/two_wheel_robot_update", two_wheel_robot_update, twoWheelRobotUpdateCallback
, (add_model_client, delete_model_client, two_wheel_robot_urdf))

# publish loop
while not rospy.is_shutdown():
	# get the wheel speed for the maintained robots
	current_robot_quantity = len(current_robots.index)
	if (robot_position_updated):
		for i in range (current_robot_quantity):
		    # get left wheel velocity
		    joint_properties_srv_msg.request.joint_name = "two_wheel_robot_" + str(current_robots.index[i]) + "::left_motor"
		    call_service = joint_properties_client.call(joint_properties_srv_msg)
		    # if call service not successful, it will leave the joint velocity unchanged
		    if (call_service):
		        if not joint_properties_srv_msg.response.success:
		            # joint not found, there is possible robot deletion the container doesn't realize
		            rospy.logwarn("there is possible robot deletion not updated")
		        else: 
		            # update the left wheel velocity
		            current_robots.left_wheel_vel[i] = joint_properties_srv_msg.response.rate[0]
		    else: 
		        rospy.logerr("fail to connect with gazebo server")
		        # do not return here
		    
		    # get right wheel velocity
		    joint_properties_srv_msg.request.joint_name = "two_wheel_robot_" + intToString(current_robots.index[i]) + "::right_motor"
		    call_service = joint_properties_client.call(joint_properties_srv_msg)
		    # if call service not successful, it will leave the joint velocity unchanged
		    if (call_service):
		        if not joint_properties_srv_msg.response.success: 
		            # joint not found, there is possible robot deletion the container doesn't realize
		            rospy.logwarn("there is possible robot deletion not updated")  
		        else:
		            # update the right wheel velocity
		            current_robots.right_wheel_vel[i] = joint_properties_srv_msg.response.rate[0]
		    else:
		        rospy.logerr("fail to connect with gazebo server")
	
		# publish the two wheel robot information container
		morph_bot_publisher.publish(current_robots)
		# reset the position update flag
		robot_position_updated = false

# update global variables
rospy.spin()
    
