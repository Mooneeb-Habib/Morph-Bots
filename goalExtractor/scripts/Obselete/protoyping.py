#!/usr/bin/python

from PIL import Image
import numpy as np 
from matplotlib import pyplot as plt
import math

plt.style.use('seaborn-whitegrid')

im = Image.open('c.png')
im = im.convert('1')
im = im.rotate(-90)
#im.show()
r,c = im.size
im_array = np.array(im)
listx = [x for x in range(0,r) for y in range(0,c) if im_array[x,y].all()==0]
listy = [y for x in range(0,c) for y in range(0,r) if im_array[x,y].all()==0]

goals = np.vstack((listx,listy))
print('goals:', goals)

#------------- Elevator Bot Goal Points

numEB = 2; #Number of Elevator Bots
EBZoneLen = int(math.floor(len(goals[1])/numEB)) #Amount of MorphBots under each EB's zone
print("EBZoneLen: ", EBZoneLen)

#---------- Separating all the morph bot goals points by quadrants defined using the mid point.
mid = [(max(listx)+min(listx))/2,(max(listy)+min(listy))/2]
print("midpoints: ",mid)

Q1 = []	#Quadrant 1 - Morph Bot right and up from the mid point
Q2 = []	#Quadrant 2 - Morph Bot left and up from the mid point
Q3 = [] #Quadrant 3 - Morph Bot left and down from the mid point
Q4 = [] #Quadrant 4 - Morph Bot right and down from the mid point
for i in range(len(goals[1])):
	if goals[0][i] > mid[0] and goals[1][i] >= mid[1]:
		Q1.append([goals[0][i],goals[1][i]])
	elif goals[0][i] <= mid[0] and goals[1][i] > mid[1]:
		Q2.append([goals[0][i],goals[1][i]])
	elif goals[0][i] < mid[0] and goals[1][i] <= mid[1]:
		Q3.append([goals[0][i],goals[1][i]])
	else:
		Q4.append([goals[0][i],goals[1][i]])

Q = []; #Combining the quadrants in an array
for i in range(len(Q1)):
	Q.append(Q1[i])
for i in range(len(Q2)):
	Q.append(Q2[i])
for i in range(len(Q3)):
	Q.append(Q3[i])
for i in range(len(Q4)):
	Q.append(Q4[i])

print("Q1:",Q1)
print("")
print("Q2:",Q2)
print("")
print("Q3:",Q3)
print("")
print("Q4:",Q4)
print("")
print("Q: ", Q)

#--------- Finding out Elevator Bot positions for each goal point
#First the EB position is prioritized to be either y+ or y- and then x+ or x-
gpEB = []

temp = 0
for i in Q1:
	tempx_r = i[0]+1
	tempy_u = i[1]+1
	if [i[0],tempy_u] not in Q:
		gpEB.append([i[0],tempy_u])
	elif [tempx_r,i[1]] not in Q:
		gpEB.append([tempx_r,i[1]])

print("gpEB1:", gpEB)

for i in Q2:
	tempx_l = i[0]-1 #value to the left of the current x-value
	tempy_u = i[1]+1 #value to the up of the current y-value
	if [i[0],tempy_u] not in Q:
		gpEB.append([i[0],tempy_u])
	elif [tempx_l,i[1]] not in Q:
		gpEB.append([tempx_l,i[1]])

print("gpEB2:", gpEB)

for i in Q3:
	tempx_l = i[0]-1 #value to the left of the current x-value
	tempy_d = i[1]-1 #value to the down of the current y-value
	if [i[0],tempy_d] not in Q:
		gpEB.append([i[0],tempy_d])
	elif [tempx_l,i[1]] not in Q:
		gpEB.append([tempx_l,i[1]])

print("gpEB3:", gpEB)

for i in Q4:
	tempx_r = i[0]+1 #value to the left of the current x-value
	tempy_d = i[1]-1 #value to the down of the current y-value
	if [i[0],tempy_d] not in Q:
		gpEB.append([i[0],tempy_d])
	elif [tempx_r,i[1]] not in Q:
		gpEB.append([tempx_r,i[1]])
print("gpEB4:", gpEB)
print("length of gpEB: ", len(gpEB))
#------------ Dividing all of the Elevator Bot goal points amongst the elevator Bot

final = []
temp = 0 #temporary variable to locate where the next gp of Elevator Bot starts
numElemFinal = 0
for i in range(numEB):
	final.insert(i,gpEB[temp:temp+EBZoneLen])
	temp += EBZoneLen
	if i == (numEB-1): 
		for listElem in final:
			numElemFinal += len(listElem)
		print("COUNT: ", numElemFinal)
		if numElemFinal < len(gpEB):
			for j in range(numElemFinal,len(gpEB)):
				final[i].append(gpEB[j])	
		# print(len(final) < len(gpEB))
		# for j in range(len(final),len(gpEB)):
		# 	final[i].append(final[i])

print("FINAL:",final)
numElemFinal = 0
for listElem in final:
	numElemFinal += len(listElem)
print("No. of elements of final: ", numElemFinal)
print("length of final: ", len(final))

# plot1 = plt.figure(1)
# for i in range(len(Q1)):
# 	plt.plot(Q1[i][0],Q1[i][1],marker='o')

# plot2 = plt.figure(2)
# for i in range(len(Q2)):
# 	plt.plot(Q2[i][0],Q2[i][1],marker='o')

# plot3 = plt.figure(3)
# for i in range(len(Q3)):
# 	plt.plot(Q3[i][0],Q3[i][1],marker='o')

# plot4c= plt.figure(4)
# for i in range(len(Q3)):
# 	plt.plot(Q4[i][0],Q4[i][1],marker='o')


plot1 = plt.figure(1)
for i in range(len(listx)):
	plt.plot(listx[i],listy[i],marker='o')
plt.plot(mid[0],mid[1],marker='o')
plt.show()



	# plot1 = plt.figure(1)
	# for i in range(len(Q1)):
	# 	plt.plot(Q1[i][0],Q1[i][1],marker='o')

	# plot2 = plt.figure(2)
	# for i in range(len(Q2)):
	# 	plt.plot(Q2[i][0],Q2[i][1],marker='o')

	# plot3 = plt.figure(3)
	# for i in range(len(Q3)):
	# 	plt.plot(Q3[i][0],Q3[i][1],marker='o')

	# plot4= plt.figure(4)
	# for i in range(len(Q3)):
	# 	plt.plot(Q4[i][0],Q4[i][1],marker='o')

	# plot5 = plt.figure(5)
	# for i in range(len(Q)):
	# 	plt.plot(Q[i][0],Q[i][1],marker='o')