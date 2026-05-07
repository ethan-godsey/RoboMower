import time
import smbus2
import math
import numpy as np
from rplidar import RPLidar
import queue
import threading
import RPi.GPIO as GPIO
from config import constants as c
from drivers import imu_trial as imu
from drivers import wheel_encoder_trial as enc

lidar_data = queue.Queue()
imu_data = queue.Queue()

GPIO.setmode(GPIO.BCM)
enc.setup_enc()

GPIO.setup(c.LIDAR_MOTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(c.LIDAR_MOTOR_PIN, 10000)
pwm.start(100)
lidar = RPLidar(c.LIDAR_PORT)
lidar.reset()
lidar.clean_input()
lidar.start_motor()

bus = smbus2.SMBus(1)

def imu_read():
    bus.write_byte_data(c.IMU_ADDR, c.PWR, 0)

    while True:
        imu_data.put((imu.read_word(c.GZ), imu.read_word(c.AX), imu.read_word(c.AY)))

def encoder_read():
    GPIO.add_event_detect(c.A_C1, GPIO.RISING, callback=enc.encoder_a_callback)
    GPIO.add_event_detect(c.B_C1, GPIO.RISING, callback=enc.encoder_b_callback)

    while True:
        time.sleep(0.001)

def lidar_read():    
    try:
        for scan in lidar.iter_scans():
            lidar_data.put(scan)
    except:
        lidar.clean_input()
        time.sleep(0.01)

try:
    imu_thread = threading.Thread(target=imu_read)
    lidar_thread = threading.Thread(target=lidar_read)
    encoder_thread = threading.Thread(target=encoder_read)
    while True:
        imu_thread.start()
        lidar_thread.start()
        encoder_thread.start()

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    GPIO.cleanup()

