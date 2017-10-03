import smbus
import time
import math
from timeit import default_timer as timer

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

	#set Gyro Offsets to 0 (we'll set our own)
	bus.write_byte_data(address, 0x13, 0x00) 
	bus.write_byte_data(address, 0x14, 0x00)
	bus.write_byte_data(address, 0x15, 0x00)
	bus.write_byte_data(address, 0x16, 0x00)
	bus.write_byte_data(address, 0x17, 0x00)
	bus.write_byte_data(address, 0x18, 0x00)
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
	time.sleep(3)

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
pitch = 0
roll = 0
frequency = 100

deg2rad = 3.1415/180  #for converting degrees to radians
rad2deg = 1/deg2rad
conv = 1/(65.5*frequency) #constant for converting gyro readings to degrees
acc_vector_mag = 0
angles_set = False #have the initial angles been set by the accelerometer
text_timer = timer()

while True:
	start = timer()
	data_g = get_Data_Gyro()
	data_a = get_Data_Acc()

	#apply gyro Calibration
	data_g['x'] = data_g['x'] - gyro_cal['x'] 
	data_g['y'] = data_g['y'] - gyro_cal['y'] 
	data_g['z'] = data_g['z'] - gyro_cal['z']

	#calculating accelerometer heading
	acc_vector_mag = math.sqrt(data_a['x']**2 + data_a['y']**2 + data_a['z']**2)
	pitch_acc = math.asin(data_a['y'] / acc_vector_mag)*rad2deg
	roll_acc  = -1*math.asin(data_a['x'] / acc_vector_mag)*rad2deg

	#calculating gyroscope heading in degrees
	pitch = pitch + data_g['x']*conv
	roll = roll + data_g['y']*conv

	pitch = pitch + roll*math.sin(data_g['z'] * conv * deg2rad)
	pitch = pitch - pitch*math.sin(data_g['z'] * conv * deg2rad)

	#we must initialize our angles to the accelerometer since it is based on a 
	#fixed frame of reference (gravity)
	if(angles_set):
		#combine gyro and acc readings with complimentary filter
		pitch = pitch*0.99 + pitch_acc*0.01
		roll = roll*0.99 + roll_acc*0.01
	else:
		#initialize readings and set flag
		pitch = pitch_acc
		roll = roll_acc
		pitch_out = pitch
		roll_out = roll
		angles_set = True

	#apply discrete time dampening
	pitch_out = pitch_out*0.9 + pitch*0.1
	roll_out  = roll_out*0.9  + roll*0.1	

	#print heading at 4hz
	if(timer() - text_timer > 0.1):
		print("Pitch: " + str(pitch_out) + "\t Roll: " + str(roll_out))
		#print(str(data_g['x'])+ "\t" + str(data_g['y']) + "\t" + str(data_g['z']))
		text_timer = timer()

	end = timer()
	interval = (1.0/ frequency) - (end - start)

	if(interval > 0): #ensure we dont wait negative time
		time.sleep(interval) #wait one time interval
	



