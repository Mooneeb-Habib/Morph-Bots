#!/usr/bin/python
import csv

def csv2gp(filename):

	Z0 = [[],[]]
	Z1 = [[],[]]
	Z2 = [[],[]]

	with open(filename, 'r') as csvfile:
		reader = csv.reader(csvfile, delimiter=',')
		for i in reader:
			Z0[0].append(i[0])
			Z0[1].append(i[1])
			Z1[0].append(i[2])
			Z1[1].append(i[3])
			Z2[0].append(i[4])
			Z2[1].append(i[5])

	return Z0,Z1,Z2