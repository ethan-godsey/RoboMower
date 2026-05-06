''' 
constants: The pinouts of the project as well as other constant values like busses and wheel diameter

Author: Ethan Godsey
Date: May 5, 2026
'''

import math

# Wheel encoder constants
CPR = 7
GEAR_RATIO = 298
TICKS_PER_REV = CPR * GEAR_RATIO
WHEEL_CIRC = 34 * math.pi
MM_PER_TICK = WHEEL_CIRC / TICKS_PER_REV

# Wheel encoder Pinouts
EN_A = 12
EN_B = 17
IN_1 = 21
IN_2 = 20
IN_3 = 16
IN_4 = 25
A_C1 = 19
A_C2 = 26
B_C1 = 6
B_C2 = 13

# IMU Ports
IMU_ADDR = 0x68
GZ = 0x47
AX = 0x3B
AY = 0x3D
PWR = 0x6B

# LiDAR pin and port
LIDAR_MOTOR_PIN = 18
LIDAR_PORT = '/dev/ttyAMA0'