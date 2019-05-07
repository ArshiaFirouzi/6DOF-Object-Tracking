# This program visualizes a 3D rectangular prism moving with 6 degrees of freedom
# Each loop of this program, calcuating and graphing the object, takes about 110ms (40ms due to pause)


import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import numpy as np
import math
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from mpl_toolkits.mplot3d import axis3d
from RectangularPrismVSD import RectPrism 
	

def refresh(RP, verts, sides, dV): # This function samples data.txt, it must do so faster than data.txt is updatated
	
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

	# Return the new vertices, sides, and displacement vector
	return verts, sides, dV

def main():
	
	# Receive rectangular prism dimensions from user and compute
	width = float(input('Rectangular Prism Width: '))
	length = float(input('Rectangular Prism Length: '))
	height = float(input('Rectangular Prism Height: '))
	RP = RectPrism(width, length, height)

	# Initialize rectangular prism and plot settings
	verts, sides, dV = RP.initialize()
	plt.ion()
	fig = plt.figure(figsize = (10,10))
	sys = fig.add_subplot(111, projection = '3d')
	maxDim = max(width,length,height)*4

	while True:

		try:
			# Extract new prims vertices, sides, and displacement
			verts, sides, dV = refresh(RP, verts, sides, dV)

			print(verts)

			sys.set_xlim(-maxDim,maxDim)
			sys.set_ylim(-maxDim,maxDim)
			sys.set_zlim(-maxDim,maxDim)
			sys.set_xlabel('X')
			sys.set_ylabel('Y')
			sys.set_zlabel('Z')
			sys.add_collection3d(Poly3DCollection(sides, facecolors = 'cyan', linewidths = 1, edgecolors = 'r', alpha = .25))

			# Pause for 40ms to avoid program crashing
			plt.draw()
			plt.pause(0.04)
			sys.clear()

		except:
			sys.clear()
			plt.close()
			break

	return 0


##########          **********          Main Functions End          **********          ##########		





##########          **********          RUN          **********          ##########		

print('~ Program Begin ~\n')
x = main()
print('\n~ Program End ~')