#!/usr/bin/python
import csv
from gpEB import *
from convList2Int import *

def csv2gp(filename, EBnum):

# This function is responsible for two tasks:
#	Task 1: Uses the csv file to obtain MorphBot Goal Points at Z0,Z1,Z2
#		Structure of goal points is as follows: mbZa -> a nested list of x and y coordinates at Z levels a = 0,1,2
#		mbZa[0] -> list of all x-coords & mbZa[1] -> list of all y-coords 
#		Example:
#			mbZ0 = [[x1,x2,x3,x4,x5,x6,x7,...],[y1,y2,y3,y4,y5,y6,y7,...]]
#	Task 2: Calls the function EBgp to extract Elevator Bot goal points using the MorphBot Goal Points.
#		The structure of goals points will be the same as that of Morph Bots
#		Example:
#			ebZ1 = [[[x1,x2,x3,x4,...],[y1,y2,y3,y4,...],[[a1,a2,a3,a4,...],[b1,b2,b3,b4,...]]]
#				where x and y (or ebZ1[0] are the coordinates for Elevator Bot 1, a and b (or EBZ1[1]) are coordinates of 
#				Elevator Bot 2 etc etc

	#------ Morphbot goal points from CSV file
	mbZ0 = [[],[]] 
	mbZ1 = [[],[]]
	mbZ2 = [[],[]]

	with open(filename, 'r') as csvfile:
		reader = csv.reader(csvfile, delimiter=',')
		for i in reader:
			mbZ0[0].append(i[0])
			mbZ0[1].append(i[1])
			mbZ1[0].append(i[2])
			mbZ1[1].append(i[3])
			mbZ2[0].append(i[4])
			mbZ2[1].append(i[5])

	#------ converting each list to int
	mbZ0 = strList2int(mbZ0)
	mbZ1 = strList2int(mbZ1)
	mbZ2 = strList2int(mbZ2)

	#---------- Elevatorbot goal points

	ebZ1 = gpEB(mbZ1,EBnum)
	ebZ2 = gpEB(mbZ2,EBnum)

	return mbZ0,mbZ1,mbZ2,ebZ1,ebZ2



