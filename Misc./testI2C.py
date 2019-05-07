# Test I2C Communications
import smbus
import time

# I2C Address of Accelerometer and Gyroscope Data
AGaddress = 0x6b

# Setup I2C
bus = smbus.SMBus(1)

# Reset Acc/Gyro of LSM9DS1
bus.write_byte_data(AGaddress, 0x22, 0x05) # Send reset message to CTRL_REG8 (location 0x22), the 0x05 byte instructs SW reset and auto-increment
time.sleep(0.05) # Give time for reset to occur

# Enable and Setup Acc/Gyro Outputs as Continuous
# No need to initialize gyroscope as default values work fine
bus.write_byte_data(AGaddress, 0x10, 0xC0) # Set Gyro Control Register (0x10) to 952Hz (3 MSBs) via 0xC0 command
bus.write_byte_data(AGaddress, 0x1F, 0x38) # Enable X,Y,Z accelerometer outputs, dnsure no decimation of outputs 
bus.write_byte_data(AGaddress, 0x20, 0xC0) # Set Acc Control Register (0x20) to 952Hz (3 MSBs) via 0xC0 command

# Read and Print Gyroscope Readings 10 times
i = 0
while i<100:
	#Read values, convert to two's complement binary, then back to decimal
	DataGX = bus.read_word_data(AGaddress, 0x18)
	DataGY = bus.read_word_data(AGaddress, 0x1A)
	DataGZ = bus.read_word_data(AGaddress, 0x1C)
	DataXX = bus.read_word_data(AGaddress, 0x28)
	DataXY = bus.read_word_data(AGaddress, 0x2A)
	DataXZ = bus.read_word_data(AGaddress, 0x2C)
#	print("Acclerometer: ", DataXX, DataXY, DataXZ)
	print("Gyroscope:    ", DataGX, DataGY, DataGZ)
	i = i+1
	time.sleep(0.01)
