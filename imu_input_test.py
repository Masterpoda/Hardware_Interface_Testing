import smbus
import time
bus = smbus.SMBus(1)
address = 0x68


def read_I2C_Word(bus, register):
	#grab high and low bytes
	high = bus.read_byte_data(address, register)
	low = bus.read_byte_data(address, register + 1)
	
	#combine high and low bytes
	word = (high << 8) + low

	#Account for signed 2's compliment
	if (word >= 0x8000):
		return -((65535-word) + 1)
	else:
		return word

def register_Setup():
	bus.write_byte_data(address, 0x6B, 0x00) #activate IMU, configure power registers
	bus.write_byte_data(address, 0x1C, 0x10) #configure accelerometer to +/- 8g range
	bus.write_byte_data(address, 0x1B, 0x08) #Configure gyro to 500 deg/s range
	return

def get_Data_Gyro():
	#gyro data is stored in sequential words starting at 0x3B
	x = read_I2C_Word(bus, 0x43)
	y = read_I2C_Word(bus, 0x45)
	z = read_I2C_Word(bus, 0x47)
	
	return {'x': x, 'y': y, 'z': z}

def get_Data_Acc():
	#gyro data is stored in sequential words starting at 0x3B
	x = read_I2C_Word(bus, 0x3B)
	y = read_I2C_Word(bus, 0x3D)
	z = read_I2C_Word(bus, 0x3F)
	
	return {'x': x, 'y': y, 'z': z}

def calibrate_Gyro():
	print("Calibrating Gyros, please set board on still surface.")
	time.sleep(5)

	#get average gyro reading when board is still.

	x_sum = 0;
	y_sum = 0;
	z_sum = 0;

	for x in range(2000):
		data = get_Data_Gyro()
		x_sum = x_sum + data['x']
		y_sum = y_sum + data['y']				
		z_sum = z_sum + data['z']				
	
	#return average value
	return {'x': x_sum/2000, 'y': y_sum/2000, 'z': z_sum/2000}

register_Setup() #call function to set up IMU registers
gyro_cal = calibrate_Gyro()

while True:
	data_g = get_Data_Gyro()
	data_a = get_Data_Acc()
	time.sleep(0.1) #wait one tenth of a second.

	"""
	# uncomment to print Gyro data
	print(str(data_g['x'] - gyro_cal['x']) + " " 
		+ str(data_g['y'] - gyro_cal['y']) + " " 
		+ str(data_g['z'] - gyro_cal['z']))
	"""


