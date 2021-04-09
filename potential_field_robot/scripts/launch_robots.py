#!/usr/bin/python

import roslaunch
import math
from random import random
from im2goal import goal_points


package = 'potential_field_robot'
executable = 'robot_state_pub.py'

goals,r,c = goal_points()
NOR = len(goals[0]) # number of robots

#print(NOR)

plotter = roslaunch.core.Node(package, 'plot_robots.py', 'plotter')
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()
process = launch.launch(plotter)

for x in range(NOR):
	name = 'robot' + str(x)
	node = roslaunch.core.Node(package, executable, name)
	launch = roslaunch.scriptapi.ROSLaunch()
	launch.start()
	process = launch.launch(node)

launch.spin()







"""uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
#pkg_gaz = 'gazebo_ros'
#exec_gaz = 'spawn_model'
#name_gaz = 'spawnminibot_model' + str(x)
#gazebo_node = roslaunch.core.Node(pkg_gaz, exec_gaz, name_gaz, args= '-model:=')
	#process = launch.launch(gazebo_node)
NOR = 16
sz = int(round(math.sqrt(NOR)))
c = 0
for i in range(sz,NOR,sz):
	for j in range(sz,NOR,sz):
		c += 1
		goals.insert(c,[i,j])

NOR_arg = str(NOR)

for x in range(NOR):
	node_name = 'name:=robot' + str(x)
	gx_pos_arg = 'gx_pos:=' + str(goals[0][x])
	gy_pos_arg = 'gy_pos:=' + str(goals[1][x])
	cli_args = ['/home/hashim/catkin_ws/src/potential_field_robot/launch/pfrobot.launch', NOR_arg, gx_pos_arg, gy_pos_arg, node_name]
	
	roslaunch_args = cli_args[1:]
	print('cli_args: ', roslaunch_args)
	roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
	parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
	parent.start()


#print process.is_alive()
#process.stop()"""
