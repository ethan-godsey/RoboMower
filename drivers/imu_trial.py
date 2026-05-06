''' 
lidar_trial: A test to get readings from a an MPU9250 IMU on a
Raspberry Pi 4B. I only take readings of roll and pitch, putting them
into a complementary filter

Author: Ethan Godsey
Date: April 3, 2026
'''

import smbus2
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

bus = smbus2.SMBus(1)

addr = 0x68

bus.write_byte_data(addr, addr, 0)

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

	# 2s complement of negative values
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

count = 0
sum_gyro_bias_x = 0
gyro_bias_x = 0
sum_gyro_bias_y = 0
gyro_bias_y = 0
sum_gyro_bias_z = 0
gyro_bias_z = 0

angle_x, angle_y, angle_z = 0,0,0
start_time = time.time()
while True:
	last_time = time.time()
	t = last_time - start_time
	
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
	
	if count < 500:
		sum_gyro_bias_x += gx
		sum_gyro_bias_y += gy
		sum_gyro_bias_z += gz
	
	if count == 500:
		gyro_bias_x = sum_gyro_bias_x / 500
		gyro_bias_y = sum_gyro_bias_y / 500
		gyro_bias_z = sum_gyro_bias_z / 500
		
	a_pitch, a_roll = compute_tilt(x, y, z)
	
	dt = time.time() - last_time
	if count > 500:
		angle_x = (angle_x + (gx * dt))
		angle_y = (angle_y + (gy * dt))
		angle_z = (angle_z + (gz * dt))

	roll = (angle_y * 0) + (a_roll * 1 )
	pitch = (angle_x * 0) + (a_pitch * 1)
	roll = a_roll
	pitch = a_pitch
	print (f"x:{roll}, y:{pitch}")
	
	'''
	roll_data.append(roll)
	pitch_data.append(pitch)
	t_data.append(t)
	ax.clear()
	ax.plot(t_data, roll_data, label="Roll")
	ax.plot(t_data, pitch_data, label="Pitch")
	
	ax.set_xlabel("Time")
	ax.set_ylabel("Degrees")
	ax.legend()
	'''
	
	#plt.pause(0.01)
	
