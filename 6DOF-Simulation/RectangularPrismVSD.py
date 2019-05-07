import numpy as np
import math

class RectPrism:

	def __init__(self, width, length, height):
		self.width = float(width)
		self.length = float(length)
		self.height = float(height)

	def initialize(self):	#Setup object
		
		# Centered at origin
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

		# Setup sides of the rectangular prism
		s1 = np.array([va,vb,vd,vc])
		s2 = np.array([vc,vd,vh,vg])
		s3 = np.array([vg,vh,vf,ve])
		s4 = np.array([ve,vf,vb,va])
		s5 = np.array([vb,vd,vh,vf])
		s6 = np.array([va,vc,vg,ve])
		sides = np.array([s1,s2,s3,s4,s5,s6])

		# Initialize the displacement vector
		dV = [0,0,0]

		# Return the rectangular prism's vertices, sides, and displacement vector
		return verts, sides, dV

	def move(self, verts, sides, dV, xMov, yMov, zMov): # Displace the object
		verts = np.array(verts)
		sides = np.array(sides)
		dV = list(dV)
		xMov = float(xMov)
		yMov = float(yMov)
		zMov = float(zMov)

		# Update displacement vector
		dV[0] = dV[0]+xMov
		dV[1] = dV[1]+yMov
		dV[2] = dV[2]+zMov

		for i in range(len(verts)):
			verts[i][0] = verts[i][0]+xMov
			verts[i][1] = verts[i][1]+yMov
			verts[i][2] = verts[i][2]+zMov

		# Setup sides
		s1 = np.array([verts[0],verts[1],verts[3],verts[2]])
		s2 = np.array([verts[2],verts[3],verts[7],verts[6]])
		s3 = np.array([verts[6],verts[7],verts[5],verts[4]])
		s4 = np.array([verts[4],verts[5],verts[1],verts[0]])
		s5 = np.array([verts[1],verts[3],verts[7],verts[5]])
		s6 = np.array([verts[0],verts[2],verts[6],verts[4]])
		sides = np.array([s1,s2,s3,s4,s5,s6])

		# Return the new vertices, sides, and displacement vector - post move
		return verts, sides, dV

	def rotate(self, verts, sides, dV, xRot, yRot, zRot):	# Rotate the object
		
		verts = np.array(verts)
		sides = np.array(sides)
		dV = list(dV)
		xRot = math.radians(float(xRot))
		yRot = math.radians(float(yRot))
		zRot = math.radians(float(zRot))

		# Create rotation matrices
		Rx = np.array([[1,0,0],[0,np.cos(xRot),np.sin(xRot)],[0,-np.sin(xRot),np.cos(xRot)]])
		Ry = np.array([[np.cos(yRot),0,-np.sin(yRot)],[0,1,0],[np.sin(yRot),0,np.cos(yRot)]])
		Rz = np.array([[np.cos(zRot),np.sin(zRot),0],[-np.sin(zRot),np.cos(zRot),0],[0,0,1]])

		# Return vertices to the origin for rotation
		for i in range(len(verts)):
			verts[i][0] = verts[i][0]-dV[0]
			verts[i][1] = verts[i][1]-dV[1]
			verts[i][2] = verts[i][2]-dV[2]

		# Rotate about X-Axis
		for i in range(len(verts)):
			verts[i] = Rx.dot(verts[i])
		# Rotate about Y-Axis
		for i in range(len(verts)):
			verts[i] = Ry.dot(verts[i])
		# Rotate about Z-Axis
		for i in range(len(verts)):
			verts[i] = Rz.dot(verts[i])

		# Return vertices to the displaced position - post rotation
		for i in range(len(verts)):
			verts[i][0] = verts[i][0]+dV[0]
			verts[i][1] = verts[i][1]+dV[1]
			verts[i][2] = verts[i][2]+dV[2]

		# Create the new sides
		s1 = np.array([verts[0],verts[1],verts[3],verts[2]])
		s2 = np.array([verts[2],verts[3],verts[7],verts[6]])
		s3 = np.array([verts[6],verts[7],verts[5],verts[4]])
		s4 = np.array([verts[4],verts[5],verts[1],verts[0]])
		s5 = np.array([verts[1],verts[3],verts[7],verts[5]])
		s6 = np.array([verts[0],verts[2],verts[6],verts[4]])
		sides = np.array([s1,s2,s3,s4,s5,s6])

		# Return the new vertices and sides - post rotation
		return verts, sides

##########          **********          RectPrism Class End          **********          ##########		