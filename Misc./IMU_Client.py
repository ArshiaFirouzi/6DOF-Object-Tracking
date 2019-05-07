# SETUP CLIENT FOR COMMUNICATIONS (Laptop)

import numpy as np
import socket
import time
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from mpl_toolkits.mplot3d import axis3d

# Setup client<->server communication line
host = '192.168.1.87' # Fixed RPi IP address
port = 5560
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host,port))

i = 0

def strTOmatrix(data):
	# string = string.split('\n')

	data = data.split('\n')
	for i in range(len(data)):
		data[i] = data[i].replace('[','')
		data[i] = data[i].replace(']','')
		data[i] = data[i].replace('[[','')
		data[i] = data[i].replace(']]','')
		data[i] = data[i].replace('\n','')
		data[i] = data[i].split(' ')
		data[i] = list(filter(None, data[i]))
		for j in range(len(data[i])):
			data[i][j] = float(data[i][j])

	return data

def vertsTOsides(verts):

	# Setup sides of the rectangular prism
	s1 = np.array([verts[0],verts[1],verts[3],verts[2]])
	s2 = np.array([verts[2],verts[3],verts[7],verts[6]])
	s3 = np.array([verts[6],verts[7],verts[5],verts[4]])
	s4 = np.array([verts[4],verts[5],verts[1],verts[0]])
	s5 = np.array([verts[1],verts[3],verts[7],verts[5]])
	s6 = np.array([verts[0],verts[2],verts[6],verts[4]])
	sides = np.array([s1,s2,s3,s4,s5,s6])

	return sides

# Setup plot 
plt.ion()
fig = plt.figure(figsize = (10,10))
sys = fig.add_subplot(111, projection = '3d')

while True:

	# Receive data
	data = s.recv(1024).decode('utf-8') 

	if data=='KILL':
		break
	elif data:
		sys.clear()
		verts = strTOmatrix(data)
		sides = vertsTOsides(verts)
		sys.set_xlim(-5,5)
		sys.set_ylim(-5,5)
		sys.set_zlim(-5,5)
		sys.set_xlabel('X')
		sys.set_ylabel('Y')
		sys.set_zlabel('Z')
		sys.add_collection3d(Poly3DCollection(sides, facecolors = 'cyan', linewidths = 1, edgecolors = 'r', alpha = .25))
		plt.draw()
		plt.pause(0.4)

s.close()