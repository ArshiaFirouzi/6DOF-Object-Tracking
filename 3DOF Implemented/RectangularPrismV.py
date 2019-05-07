import numpy as np
import math

class RectPrismV:

	def __init__(self, width, length, height):
		self.width = float(width)
		self.length = float(length)
		self.height = float(height)

	def initialize(self):	#Setup object
		
		# The prism is centered at origin 
		xWidth = float(self.width)/2
		yLength = float(self.length)/2
		zHeight = float(self.height)/2

		# Setup vertices of the rectangular prism
		va = np.array([-xWidth,-yLength,zHeight])
		vb = np.array([-xWidth,-yLength,-zHeight])
		vc = np.array([xWidth,-yLength,zHeight])
		vd = np.array([xWidth,-yLength,-zHeight])
		ve = np.array([-xWidth,yLength,zHeight])
		vf = np.array([-xWidth,yLength,-zHeight])
		vg = np.array([xWidth,yLength,zHeight])
		vh = np.array([xWidth,yLength,-zHeight])
		verts = np.array([va,vb,vc,vd,ve,vf,vg,vh])

		return verts

	def rotate(self, verts, xRot, yRot, zRot):	# Rotate the object
		
		verts = np.array(verts)
		xRot = math.radians(xRot)
		yRot = math.radians(yRot)
		zRot = math.radians(zRot)

		# Create rotation matrices
		Rx = np.array([[1,0,0],[0,np.cos(xRot),np.sin(xRot)],[0,-np.sin(xRot),np.cos(xRot)]])
		Ry = np.array([[np.cos(yRot),0,-np.sin(yRot)],[0,1,0],[np.sin(yRot),0,np.cos(yRot)]])
		Rz = np.array([[np.cos(zRot),np.sin(zRot),0],[-np.sin(zRot),np.cos(zRot),0],[0,0,1]])

		# Rotate about X-Axis
		for i in range(len(verts)):
			verts[i] = Rx.dot(verts[i])
		# Rotate about Y-Axis
		for i in range(len(verts)):
			verts[i] = Ry.dot(verts[i])
		# Rotate about Z-Axis
		for i in range(len(verts)):
			verts[i] = Rz.dot(verts[i])

		return verts