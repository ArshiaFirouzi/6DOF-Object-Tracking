# Visualize IMU Orientation via Gyroscopic Readings

import smbus
import time
import socket
import math
import numpy as np



# Begin Server Class *************************************************************************************************************

class Server: 

	def __init__(self): # UDP communication

		# Setup server
		self.host = ''
		self.port = 5560
		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Setup socket
		try:
			self.s.bind((self.host,self.port)) # Bind socket
		except socket.error as msg:
			print("Socket Binding Error!")
		# print("Server Setup")

		# Setup client communication line
		self.s.listen(1) # Allows one connection at a time
		self.conn, self.address = self.s.accept()
		# print('Client Connected')

	def dataTransfer(self, conn): #NEED TO SYNC SEND/RECEIVE, maybe a function sends data every n iterations and another checks for kill every n^2

		# Receive data
		data = conn.recv(1024) # 1024 bits per data-transfer, lower for faster communications
		command = data.decode('utf-8') # Decoding in utf-8 as communications will be treated as strings

		if command == 'RPO': # Requesting the rectangular prism's orientation
			reply = "ORIENTATION DATA"
		elif command == 'KILL':
			print('Server is shutting down')
			self.s.close()
		else:
			reply = 'Unkown Command'

		# Send reply to client
		conn.sendall(str.encode(reply))

		conn.close()

	def sendData(self, data, conn):
		conn.sendall(str.encode(data))

	def killServer(self, conn):
		conn.close()


# End Server Class ***************************************************************************************************************



# Begin IMU/LSM9DS1 Class ********************************************************************************************************

class IMU:

	def __init__(self):
		# I2C address of Accelerometer and Gyroscope
		# Magnetometer has a separate address
		self.AGaddress = 0x6b

		#Setup I2C bus
		self.bus = smbus.SMBus(1)

		# Reset Acc/Gyro software of LSM9DS1
		self.bus.write_byte_data(self.AGaddress, 0x22, 0x05) # Send reset message to CTRL_REG8 (location 0x22), set to auto-increment
		time.sleep(0.05) # Give time for reset to occur

		# The accelerometer is ignored as without GPS and a Kalman Filter, position tracking is hopeless (compounding error - drift)
		# The gyroscope is scaled for rotation up to +/-245 degrees/s, most settings left to default
		# Scaling can be altered via the changing the command to control register 1 (0x10) below
		self.bus.write_byte_data(self.AGaddress, 0x10, 0xC0) # Set Gyro to 952Hz (3 MSBs) via 0xC0 command

	# 16-bit Positive Decimal to Two's Complemnt to +/- 15-bit Decimal
	def DecodeWord(self, Decimal): 

		#Number of bits in an incoming word
		Bits = 16

		#Convert decimal to binary string
		bDec = bin(Decimal).replace("0b","")

		# If len binary < span, the MSB is 0 so it is a positive number, return the decimal (as a negative, clockwise rotation is positve)
		if len(bDec)<Bits:
			return -Decimal

		# If len == Bits, the 16th position contains a 1, so it is a negative number
		# Convert the remaining 15 bits (discluding the MSB) into a negative number
		elif len(bDec)==Bits:
			nDec = int(bDec[1::],2)-(2**(Bits-1))
			if nDec == -65536:
				return "" # Could mean either maxima, DROP the information from this run
			return -nDec

		elif int(bDec,2)==2**Bits:
			return 0

		else:
			return "ERROR"

	# Take current gyroscope x/y/z data (in this case it is scaled from 0-255 degrees/second)
	def sampleDPS(self): 

		# Read unscaled values, convert to two's complement binary, then back to decimal
		DataGX = self.DecodeWord(self.bus.read_word_data(self.AGaddress, 0x18)) # Gyr X-Axis
		DataGY = self.DecodeWord(self.bus.read_word_data(self.AGaddress, 0x1A)) # Gyr Y-Axis
		DataGZ = self.DecodeWord(self.bus.read_word_data(self.AGaddress, 0x1C)) # Gyr Z-Axis

		xDPS = (DataGX/(2**15))*255
		yDPS = (DataGY/(2**15))*255
		zDPS = (DataGZ/(2**15))*255

		return xDPS, yDPS, zDPS

	# Calculate error when IMU is completely still
	def gyrERR(self):
		
		xErrL = list()
		yErrL = list()
		zErrL = list()
		i = 0
		j = 0

		# Allow readings to stabilize
		while i<300:
			xDPS, yDPS, zDPS = gyro.sampleDPS()
			i+=1

		# Take error readings
		while j<300:
			xDPS, yDPS, zDPS = gyro.sampleDPS()
			xErrL.append(xDPS)
			yErrL.append(yDPS)
			zErrL.append(zDPS)
			j+=1

		xERR = np.median(xErrL)
		yERR = np.median(yErrL)
		zERR = np.median(zErrL)

		print("Error Determined")
		return xERR, yERR, zERR

	# Convert time and DPS to degrees rotated
	def degreesRot(self, dt, xDPS, yDPS, zDPS):

		# dt * DPS = degrees rotated
		xRot = dt*xDPS
		yRot = dt*yDPS
		zRot = dt*zDPS

		return xRot, yRot, zRot

# End IMU/LSM9DS1 Class **********************************************************************************************************



# Begin Rectangular Prism Class **************************************************************************************************
# Class has been edited to remove movement (displacement vector and move function mainly)

class RectPrism:

	def __init__(self, width, length, height):
		self.width = float(width)
		self.length = float(length)
		self.height = float(height)

	def initialize(self):	#Setup object
		
		# Centered at origin so extends 1/2 of measurement along each direction/axis
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
		# s1 = np.array([va,vb,vd,vc])
		# s2 = np.array([vc,vd,vh,vg])
		# s3 = np.array([vg,vh,vf,ve])
		# s4 = np.array([ve,vf,vb,va])
		# s5 = np.array([vb,vd,vh,vf])
		# s6 = np.array([va,vc,vg,ve])
		# sides = np.array([s1,s2,s3,s4,s5,s6])

		# Return the rectangular prism's vertices and sides
		return verts#, sides

	def rotate(self, verts, xRot, yRot, zRot):	# Rotate the object
		
		verts = np.array(verts)
		# sides = np.array(sides)
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

		# Create the new sides
		# s1 = np.array([verts[0],verts[1],verts[3],verts[2]])
		# s2 = np.array([verts[2],verts[3],verts[7],verts[6]])
		# s3 = np.array([verts[6],verts[7],verts[5],verts[4]])
		# s4 = np.array([verts[4],verts[5],verts[1],verts[0]])
		# s5 = np.array([verts[1],verts[3],verts[7],verts[5]])
		# s6 = np.array([verts[0],verts[2],verts[6],verts[4]])
		# sides = np.array([s1,s2,s3,s4,s5,s6])

		# Return the new vertices and sides - post rotation
		return verts#, sides

# End Rectangular Prism Class ****************************************************************************************************



#Setup Objects and Server
gyro = IMU()
RP = RectPrism(1,3,0.1)
verts = RP.initialize()
print("Objects Initialized")
serv = Server()
print("Server Connected")

# Send client initial coordinates of RP
serv.sendData(str(verts), serv.conn) # Apx 4ms

# Calculate error in gyro readings
xERR, yERR, zERR = gyro.gyrERR()
# xERR = 0
# yERR = 0
# zERR = 0
# Calculate motion of RP via the IMU
j = 0
i = 0

while j<10000:
	# Sample the gyroscope
	xDPS, yDPS, zDPS = gyro.sampleDPS()
	# Allow readings to stabilize
	if i<300:
		i+=1
		continue
	if j==0:
		t0 = time.time()
	# Adjust gyro readings
	xDPS = xDPS-xERR
	yDPS = yDPS-yERR
	zDPS = zDPS-zERR
	# Set time variables
	t1 = time.time()
	dt = t1-t0
	t0 = t1
	# Extract degrees rotated from gryo readings (DPS to degrees)
	xRot, yRot, zRot = gyro.degreesRot(dt, xDPS, yDPS, zDPS)
	# Calculate new RP coordinates
	verts = RP.rotate(verts, xRot, yRot, zRot)
	if j%500==0:
		serv.sendData(str(verts), serv.conn) # Apx 4ms
	j+=1

# print(np.sum(xRL), np.sum(yRL), np.sum(zRL))
print("Process Complete, Shutting Down")
serv.sendData('KILL', serv.conn)
serv.killServer(serv.conn)