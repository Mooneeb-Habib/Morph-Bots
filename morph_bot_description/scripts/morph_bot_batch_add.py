#!/usr/bin/python

# this node add a number of two wheel robots by talking to the manager node
# through the "/morph_sim/two_wheel_robot_update" service

# work as a supplementary function of the two wheel robot manager node
# when batch adding, a distribution range needs to specified

import rospy
import sys
from swarm_robot_srv.srv import two_wheel_robot_update

rospy.init_node('morph_bot_batch_add')

# handshake with robot name in parameter server
robot_name = rospy.get_param('/morph_sim/robot_name')

if (not robot_name):
	rospy.logerr("simulation environment(parameters) is not set")
	sys.exit()

if (robot_name != "morph_bot"):
	rospy.logerr("wrong robot according to parameter server")
	sys.exit()

# check if service is ready, "/morph_sim/two_wheel_robot_update"

rospy.wait_for_service("/morph_sim/two_wheel_robot_update")
rospy.loginfo("/morph_sim/two_wheel_robot_update service is ready")

# instantiate a service client
two_wheel_robot_update_client = rospy.ServiceProxy("two_wheel_robot_update", two_wheel_robot_update)
two_wheel_robot_update_srv_msg = two_wheel_robot_update()

# get the private parameters passed to this node
# parameter: robot_quantity
rospy.loginfo(rospy.get_param_names())
robot_quantity = rospy.get_param("/morph_sim/morph_bot_batch_add/robot_quantity")
if (robot_quantity): 
	rospy.loginfo("using robot quantity passed in: %s" % robot_quantity)
	rospy.delete_param("/morph_sim/morph_bot_batch_add/robot_quantity")
else:
	robot_quantity = 10  # the default value
	rospy.loginfo("using default robot quantity: 10")
    
# parameter: half_range

half_range = rospy.get_param("/morph_sim/morph_bot_batch_add/half_range")
if (half_range):
    rospy.loginfo("using half range passed in: %s" % half_range)
    rospy.delete_param("/morph_sim/morph_bot_batch_add/half_range")
else:
	half_range = 1.0  # the default value
	rospy.loginfo("using default half range: 1.0")

# prepare the service message
two_wheel_robot_update_srv_msg.update_code = 1
two_wheel_robot_update_srv_msg.add_mode = two_wheel_robot_update_srv_msg.ADD_MODE_RANDOM
two_wheel_robot_update_srv_msg.half_range = half_range

# call service to add robot repeatedly
call_service = False
minimal_delay = rospy.Duration(0.01)   # this is a moderate waiting time

for i in range(robot_quantity):
    # call the service to add one robot randomly
    call_service = two_wheel_robot_update_client.call(two_wheel_robot_update_srv_msg)

    if (call_service):
    	response_code = two_wheel_robot_update_srv_msg.response_code
        if (response_code == two_wheel_robot_update_srv_msg.SUCCESS):
            rospy.loginfo("success")
            break
        if (response_code == two_wheel_robot_update_srv_msg.ADD_FAIL_NO_RESPONSE):
            rospy.logwarn("add fail because no response from gazebo")
            break
        if (response_code == two_wheel_robot_update_srv_msg.ADD_FAIL_TOO_CROWDED):
            rospy.logwarn("add fail because the range is too crowded")
            break
        if (response_code == two_wheel_robot_update_srv_msg.FAIL_OTHER_REASONS):
            # keep trying for this case
            rospy.logwarn("fail because of other reasons")
            while (call_service != two_wheel_robot_update_client.call(two_wheel_robot_update_srv_msg)):
                rospy.logwarn("keep trying to add one robot")
                minimal_delay.sleep()
            rospy.loginfo("success after retry")
            break
        else:
            # there is no reason to be here, otherwise there is problems in service request
            rospy.logerr("wrong response code, check the service request for details")
    
    else: 
        rospy.logerr("fail to connect to service /morph_sim/two_wheel_robot_update")
    # delay for a minimal time, so that topic message in manager can update
    minimal_delay.sleep()


