from gpExtractor import *
from matplotlib import pyplot as plt

f = 'morphGoalPoint-pentagon.csv' #the CSV file containing gp for morph bot

Elev_Bot_num = 4	# the Number of ELevator Bots

mbZ0,mbZ1,mbZ2,ebZ1,ebZ2 = csv2gp(f,Elev_Bot_num)

# print("THIS IS mbZ0: ",mbZ0)
# print("")
print("THIS IS mbZ1: ",mbZ1)
print("")
# print("This is mbZ2: ",mbZ2)
# print("")
# print("This is ebZ1: ",ebZ1)
# print("")
# print("This is ebZ2: ",ebZ2)
# print("")

plot6 = plt.figure(6)
for i in range(len(mbZ1[0])):
	plt.plot(mbZ0[0][i],mbZ0[1][i],marker='o')
#plt.plot(3.5,3.5,marker='o')
plt.show()