#!/usr/bin/python
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
plt.style.use('seaborn-whitegrid')

def goal_points():
	im = Image.open('/home/hashim/catkin_ws/src/morph_bot_sf/images/Flower.png')
	im = im.convert('1')
	r,c = im.size
	im_array = np.array(im)
	listx = [x for x in range(0,r) for y in range(0,c) if im_array[x,y].all() == 0]
	listy = [y for x in range(0,r) for y in range(0,c) if im_array[x,y].all() == 0]
	listx.reverse()
	#plt.scatter(listx,listy,marker ='o')
	#plt.show()
	mid = [float(max(listy)+min(listy))/2,float(max(listx)+min(listx))/2]
	goals = np.vstack((listy, listx))
	return goals, mid, r, c

