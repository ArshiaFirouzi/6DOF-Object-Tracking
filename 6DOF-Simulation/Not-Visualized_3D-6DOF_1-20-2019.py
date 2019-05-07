# This program performs the tracking portion of 3D_Visualization.py (rectangular prism w/ 6DOF), except it doesn't graph the object
# We can see that each loop of this program takes about 5ms (20ms due to pause), 4.5% the time for each loop of 3D_Visualization.py
# The reduction in time is most likely due to the graphing done in each loop of Visualized_3D-6DOF.py


import time
import numpy as np
import math
from RectangularPrismVSD import RectPrism 



##########          **********          Functions Begin          **********          ##########		

def refresh(RP, verts, sides, dV): # This function must sample faster than data.txt is updatated
	
	# Initialize arrays for the rectangular prism's vertices, sides, and displacement
	verts = np.array(verts)
	sides = np.array(sides)
	dV = np.array(dV)

	# Pull command data from a separate document, the command data document (data.txt) must be updated faster than each loop of this one
	pullData = open('data.txt','r').read()
	lines = pullData.split('\n')

	# Find most recent command, skip empty lines
	i = -1
	while True:
		if len(lines[i]) > 0:
			newData = lines[i]
			break
		else:
			i=i-1
			continue

	# Extract the line's data
	# 'DisplacementData.csv ~ RotationData.csv' is the data's per line format
	movData, rotData = newData.split('~')
	movData = movData.split(',')
	rotData = rotData.split(',')

	# Rx, Ry, Rz is the rotation's directional data format
	RotateX = rotData[0]
	RotateY = rotData[1]
	RotateZ = rotData[2]
	verts, sides = RP.rotate(verts, sides, dV, RotateX, RotateY, RotateZ)

	# Mx, My, Mz is the movement's directional data format
	MoveX = movData[0]
	MoveY = movData[1]
	MoveZ = movData[2]
	verts, sides, dV = RP.move(verts, sides, dV, MoveX, MoveY, MoveZ)
	
	# Return the new vertices, sides, displacement vector
	return verts, sides, dV

def main():
	
	# Receive rectangular prism dimensions from user and compute
	width = float(input('Rectangular Prism Width: '))
	length = float(input('Rectangular Prism Length: '))
	height = float(input('Rectangular Prism Height: '))
	RP = RectPrism(width, length, height)

	# Initialize rectangular prism
	verts, sides, dV = RP.initialize()

	# Initialize time
	refreshTime = list() # Make sure to remove refreshTime[0] as it is the "initialization" time 
	t = time.time()
	print(verts) # Initial vertices of the rectangular prism

	i = 0
	while i<50:

		# Store and update time
		# refreshTime.append(time.time()-t)
		# t = time.time()

		# Extract new vertices, sides, displacement
		verts, sides, dV = refresh(RP, verts, sides, dV)

		print(verts)

		# Pause for 20ms to avoid program crashing
		time.sleep(0.02)

		i = i + 1

	# refreshTime.pop(0) # Remove initialization time
	# return refreshTime

##########          **********          Main Functions End          **********          ##########		





##########          **********          RUN          **********          ##########		

print('~ Program Begin ~\n')
main()
#print(np.mean(refreshTime))
#print(f'\n{refreshTime}')
print('\n~ Program End ~')