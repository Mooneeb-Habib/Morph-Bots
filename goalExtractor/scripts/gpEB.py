#!/usr/bin/python
import csv
import math
from matplotlib import pyplot as plt

#Converts the sorted EB goal points into the same convention as that of MB goal points.
def gpConvention(gpEB_sorted):
	gpEB_organized = [] #EB goal points organized in the same fashion as MB goal points
	print("gpEB_sorted: ", gpEB_sorted)
	for i in range(len(gpEB_sorted)):
		temp1 = [j[0] for j in gpEB_sorted[i]]
		temp2 = [j[1] for j in gpEB_sorted[i]]
		gpEB_organized.insert(i,[temp1,temp2])
	return gpEB_organized

#Sorts out what goal points fall under what EB. 
def gpSorter(ebList, numEB, MBperEB):
	gpEB_sorted = [] #goal points divided amongst the Elevator Bot 
	temp = 0 #temporary variable to locate where the next goal point of Elevator Bot starts
	numElem = 0 #number of elements in gpEB_sorted
	for i in range(numEB):
		gpEB_sorted.insert(i,ebList[temp:temp+MBperEB])
		temp += MBperEB
		if i == (numEB-1): 
			for listElem in gpEB_sorted:
				numElem += len(listElem)
			if numElem < len(ebList): #Check to see if all goal-points are divided amongst the EB
				for j in range(numElem,len(ebList)):
					gpEB_sorted[i].append(ebList[j])

	ebListFinal = gpConvention(gpEB_sorted)

	return ebListFinal

#The main function that is called to use the Morphbot Goal Points to calculate EB goal Points
def gpEB(mbZa,numEB):

	#---------------- For Z = 1
	MBperEB = int(math.floor(len(mbZa[1])/numEB)) #Min No. of MB that each EB's is responsible for for Z =1 
	print("MBperEB: ", MBperEB)
	#---------- Separating the Morph Bot goal points into 4 quadrants
	mid = [float(max(mbZa[0])+min(mbZa[0]))/2,float(max(mbZa[1])+min(mbZa[1]))/2] #the mid point of the contour at Z = 1
	print("")
	print("mid: ", mid)
	Q1 = []	#Quadrant 1 - Morph Bot right and up from the mid point
	Q2 = []	#Quadrant 2 - Morph Bot left and up from the mid point
	Q3 = [] #Quadrant 3 - Morph Bot left and down from the mid point
	Q4 = [] #Quadrant 4 - Morph Bot right and down from the mid point
	for i in range(len(mbZa[1])):
		if mbZa[0][i] > mid[0] and mbZa[1][i] >= mid[1]:
			Q1.append([mbZa[0][i],mbZa[1][i]])
		elif mbZa[0][i] <= mid[0] and mbZa[1][i] > mid[1]:
			Q2.append([mbZa[0][i],mbZa[1][i]])
		elif mbZa[0][i] < mid[0] and mbZa[1][i] <= mid[1]:
			Q3.append([mbZa[0][i],mbZa[1][i]])
		else:
			Q4.append([mbZa[0][i],mbZa[1][i]])

	Q = []; #Combining the quadrants in an array
	for i in range(len(Q1)):
		Q.append(Q1[i])
	for i in range(len(Q2)):
		Q.append(Q2[i])
	for i in range(len(Q3)):
		Q.append(Q3[i])
	for i in range(len(Q4)):
		Q.append(Q4[i])

	# --------- Calculating Elevator Bot positions for each goal point
	#The first priority is to place EB either +y or -y of the morphbot position. If not possible then +x or -x.

	gpEB_unsorted = [] #The goalpoints for all Elevator Bots

	for i in Q1:
		tempx_r = i[0]+1
		tempy_u = i[1]+1
		if [i[0],tempy_u] not in Q:
			gpEB_unsorted.append([i[0],tempy_u])
		elif [tempx_r,i[1]] not in Q:
			gpEB_unsorted.append([tempx_r,i[1]])

	for i in Q2:
		tempx_l = i[0]-1 #value to the left of the current x-value
		tempy_u = i[1]+1 #value to the up of the current y-value
		if [i[0],tempy_u] not in Q:
			gpEB_unsorted.append([i[0],tempy_u])
		elif [tempx_l,i[1]] not in Q:
			gpEB_unsorted.append([tempx_l,i[1]])

	for i in Q3:
		tempx_l = i[0]-1 #value to the left of the current x-value
		tempy_d = i[1]-1 #value to the down of the current y-value
		if [i[0],tempy_d] not in Q:
			gpEB_unsorted.append([i[0],tempy_d])
		elif [tempx_l,i[1]] not in Q:
			gpEB_unsorted.append([tempx_l,i[1]])

	for i in Q4:
		tempx_r = i[0]+1 #value to the left of the current x-value
		tempy_d = i[1]-1 #value to the down of the current y-value
		if [i[0],tempy_d] not in Q:
			gpEB_unsorted.append([i[0],tempy_d])
		elif [tempx_r,i[1]] not in Q:
			gpEB_unsorted.append([tempx_r,i[1]])

	#------------ Dividing all of the Elevator Bot goal points amongst the elevator Bot

	ebZa = gpSorter(gpEB_unsorted, numEB, MBperEB)
	print("ebZa: ", ebZa)

	return ebZa