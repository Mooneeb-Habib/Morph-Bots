#!/usr/bin/python
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
plt.style.use('seaborn-whitegrid')

def goal_points():
	im = Image.open('/home/hashim/catkin_ws/src/potential_field_robot/scripts/dots.png')
	im = im.convert('1')
	r,c = im.size
	im_array = np.array(im)
	listx = [x for x in range(0,r) for y in range(0,c) if im_array[x,y].all() == 0]
	listy = [y for x in range(0,r) for y in range(0,c) if im_array[x,y].all() == 0]
	#plt.scatter(listx,listy,marker ='o')
	#plt.show()
	goals = np.vstack((listx, listy))
	return goals, r, c

