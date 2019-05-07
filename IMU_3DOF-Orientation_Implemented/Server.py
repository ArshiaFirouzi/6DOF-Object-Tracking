# Visualize IMU Orientation via Gyroscopic Readings

import smbus
import time
import socket
import math
import numpy as np
from RectangularPrismV import RectPrismV



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
			print(msg)
		# print("Server Setup")

		# Setup client communication line
		self.s.listen(1) # Allows one connection at a time
		self.conn, self.address = self.s.accept()
		# print('Client Connected')

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

		# The accelerometer is ignored as without GPS and a Kalman Filter, position tracking is hopeless (compounding error, drift)
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
		while i<500:
			xDPS, yDPS, zDPS = gyro.sampleDPS()
			i+=1

		# Take error readings
		while j<1000:
			xDPS, yDPS, zDPS = gyro.sampleDPS()
			xErrL.append(xDPS)
			yErrL.append(yDPS)
			zErrL.append(zDPS)
			j+=1

		xERR = np.mean(xErrL)
		yERR = np.mean(yErrL)
		zERR = np.mean(zErrL)

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


def main():

	#Setup Vertices and Server
	verts = RP.initialize()
	print("Objects Initialized")
	serv = Server()
	print("Server Connected")

	# Send client initial coordinates of RP
	serv.sendData(str(verts), serv.conn) # Apx 4ms

	# Calculate error in gyro readings
	xERR, yERR, zERR = gyro.gyrERR()

	# Calculate motion of the RP via from IMU data
	j = 0
	t0 = time.time()
	while j<10000:
		# Sample the gyroscope
		xDPS, yDPS, zDPS = gyro.sampleDPS()

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
		if j%100==0:
			serv.sendData(str(verts), serv.conn) # Apx 4ms
		j+=1

	print("Process Complete, Shutting Down")
	serv.sendData('KILL', serv.conn)
	time.sleep(1)
	serv.killServer(serv.conn)

	return 0





##########          **********          RUN          **********          ##########		

print('~ Program Begin ~\n')
gyro = IMU()
RP = RectPrismV(1,3,0.1)
x = main()
print('\n~ Program End ~')