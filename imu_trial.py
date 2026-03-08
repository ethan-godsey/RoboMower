import smbus2
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

bus = smbus2.SMBus(1)
addr = 0x68

bus.write_byte_data(addr, 0x68, 0)

# for the accelerometer, good but not good with vibrations / shaking
def compute_tilt(x, y, z):
	roll = math.atan2(y, z)
	pitch = math.atan2(-x, math.sqrt(y*y + z*z))
	roll = math.degrees(roll)
	pitch = math.degrees(pitch)
	
	return roll, pitch
	
def read_word(reg):
	high = bus.read_byte_data(addr, reg)
	low = bus.read_byte_data(addr, reg+1)
	val = (high << 8) + low
	if val >= 0x8000:
		val = -((65535 - val) + 1)
	return val

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plt.ion()
fig, ax = plt.subplots()
	
roll_data = []
pitch_data = []
t_data = []
start_time = time.time()

count = 0
sum_gyro_bias_x = 0
gyro_bias_x = 0
sum_gyro_bias_y = 0
gyro_bias_y = 0
sum_gyro_bias_z = 0
gyro_bias_z = 0

angle_x, angle_y, angle_z = 0,0,0

while True: 
	count+= 1
	ax_val = read_word(0x3B)
	ay_val = read_word(0x3D)
	az_val = read_word(0x3F)
	
	# get the values and normalize them
	gx = (read_word(0x43)/131) - gyro_bias_x
	gy = (read_word(0x45)/131) - gyro_bias_y
	gz = (read_word(0x47)/131) - gyro_bias_z
	
	x = ax_val/16384
	y = ay_val/16384
	z = az_val/16384
	
	if count < 1000:
		sum_gyro_bias_x += gx
		sum_gyro_bias_y += gy
		sum_gyro_bias_z += gz
	
	if count == 1000:
		gyro_bias_x = sum_gyro_bias_x / 1000
		gyro_bias_y = sum_gyro_bias_y / 1000
		gyro_bias_z = sum_gyro_bias_z / 1000
		
	roll, pitch = compute_tilt(x, y, z)
	
	t = time.time() - start_time
	if count > 1000:
		angle_x = (angle_x + (gx * (t/100000000)))
		angle_y = (angle_y + (gy * (t/100000000)))
		angle_z = (angle_z + (gz * (t/100000000)))
	
	start_time = t
	#old_gx = gx
	#old_gz = gz
	#old_gy = gy

	print (f"x:{angle_x}, y:{angle_y}, z:{angle_z}")
	#print (f"x:{gx}, y:{gy}, z:{gz}")
	roll_data.append(roll)
	pitch_data.append(pitch)
	t_data.append(t)
	ax.clear()
	ax.plot(t_data, roll_data, label="Roll")
	ax.plot(t_data, pitch_data, label="Pitch")
	
	ax.set_xlabel("Time")
	ax.set_ylabel("Degrees")
	ax.legend()
	
	plt.pause(0.01)
	
