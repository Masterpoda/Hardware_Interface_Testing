import smbus
import time
bus = smbus.SMBus(0)
address = 0x68

def read_I2C_Word(bus, register)
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

def register_Setup()
	bus.write_byte_data(address, 0x6B) #activate IMU, configure power registers
	bus.write_byte_data(address, 0x00)
	bus.write_byte_data(address, 0x1C) #configure accelerometer to +/- 8g range
	bus.write_byte_data(address, 0x10)
	bus.write_byte_data(address, 0x1B) #Configure gyro to 500 deg/s range
	bus.write_byte_data(address, 0x08)
	return

def get_Data_Gyro()
	
	return {'x': x, 'y': y, 'z': z}
