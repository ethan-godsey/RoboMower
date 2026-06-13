import time
import smbus2
import math
import numpy as np
from rplidar import RPLidar
import queue
import threading
import multiprocessing
import RPi.GPIO as GPIO
import collections
import ekf as ekf
from config import constants as c
from drivers import imu_trial as imu
from drivers import wheel_encoder_trial as enc
from software import pi_to_laptop as front_end

# global starting variables for EKF
global start
state = np.array([0.0, 0.0, 0.0])
p = np.zeros((3, 3))


lndmrks = [(1, 0)]
prev_dist = 0.0

'''func: imu_read: get output from IMU and put into queue'''
def imu_read():
    # Activate IMU
    sum_gyro_bias = 0.0
    bus.write_byte_data(c.IMU_ADDR, c.PWR, 0)

    # Gather sample of bias while LiDAR spins up
    for i in range(500):
        gz = imu.read_word(c.GZ)
        sum_gyro_bias += gz
    
    # average out bias to correct later
    gyro_bias_z = sum_gyro_bias / 500

    # add corrected head value to queue with x and y from accelerometer
    while True:
        unbiased_gz = imu.read_word(c.GZ) - gyro_bias_z
        imu_data.append((unbiased_gz, imu.read_word(c.AX), imu.read_word(c.AY)))


'''func: encoder_read: get output of encoder based on when event being triggered'''
def encoder_read():
    GPIO.add_event_detect(c.A_C1, GPIO.RISING, callback=enc.encoder_a_callback)
    GPIO.add_event_detect(c.B_C1, GPIO.RISING, callback=enc.encoder_b_callback)

    while True:
        time.sleep(0.001)

'''func: lidar_read: get output of lidar spin, and then put in queue'''
def lidar_read(queue):
    lidar = RPLidar(c.LIDAR_PORT)
    lidar.reset()
    lidar.clean_input()
    lidar.start_motor()    
    while True:
        try:
            for scan in lidar.iter_scans(min_len=5):
                queue.put(scan)        
        except:
            lidar.clean_input()

def move_fwd():
    enc.forward(100)
    time.sleep(3)
    enc.stop()

'''Control loop that goes to a landmark given state'''
def navigate_to(tx, ty, state):
    # calculate distance in 2D space to landmark
    thresh = 1.20
    dx = tx - state[0]
    dy = ty - state[1]
    dist = math.sqrt(dx**2 + dy**2)

    base_speed = 60
    gain = 60

    # calculate heading difference
    heading_err = ((math.atan2(dy, dx) - state[2] + math.pi) % (2 * math.pi)) - math.pi


    left_speed  = max(0, min(100, base_speed + gain * heading_err))
    right_speed = max(0, min(100, base_speed - gain * heading_err))
    
    
    if dist < 0.05:
        return True
    if abs(heading_err) > thresh:
        if heading_err > 0:
            right_speed = -1 * left_speed
        else:
            left_speed = -1 * right_speed
    
    if abs(heading_err) < 0.05:
        enc.forward(base_speed)
    else:
        enc.drive(right_speed, left_speed)
    return False

if __name__ == '__main__':
    try:
        # queue is filling up too fast for LiDAR and Python GIL bogs threads down, so starting a process for LiDAR
        multiprocessing.set_start_method('spawn')

        # queues to get most recent data from varying feedback speeds
        lidar_data = multiprocessing.Queue()
        imu_data = collections.deque(maxlen=20)
        wf_queue = multiprocessing.Queue()

        # setup IMU
        bus = smbus2.SMBus(1)
        
        # setup encoder
        GPIO.setmode(GPIO.BCM)
        enc.setup_enc()

        # setup LiDAR
        GPIO.setup(c.LIDAR_MOTOR_PIN, GPIO.OUT)
        pwm = GPIO.PWM(c.LIDAR_MOTOR_PIN, 10000)
        pwm.start(50)

        # start the threads and set variables for loop
        delta_thet = 0.0
        imu_thread = threading.Thread(target=imu_read)
        encoder_thread = threading.Thread(target=encoder_read)
        lidar_reader = multiprocessing.Process(target=lidar_read, args=(lidar_data,))
        front = multiprocessing.Process(target=front_end.main_wrap, args=(wf_queue,))
        there = False
        done = False
        imu_thread.start()
        lidar_reader.start()
        encoder_thread.start()
        front.start()

        # initial time to reference
        time.sleep(5)

        while True:
        
            # get imu header data to make moving average of noise (nothing else is controlled in the meantime)
            latest_imu = None
            recent_scan = None
            count = 0
            imu_sum = 0
            for meas in list(imu_data):
                imu_sum += meas[0]
                count += 1

            if count != 0:
                delta_thet = imu_sum / count
            
            while not lidar_data.empty():
                recent_scan = lidar_data.get_nowait()
                
            # get encoder distance (ut)
            current_dist = (enc.dist_a + enc.dist_b) / 2000
            dist = current_dist - prev_dist
            prev_dist = current_dist

            # get new predicted position and uncertainty matrix and view state
            state, p = ekf.predict(state, p, dist, delta_thet) 

            if recent_scan:
                state, p = ekf.update(state, p, recent_scan, lndmrks)
                wf_queue.put({"scan": recent_scan, "state": state.tolist()})
            # print(state)

            # get measurements from start to landmark
            distance = math.sqrt((state[0] - 1)**2 + (state[1] - 1)**2)
            head = ((math.atan2(1 - state[1], 1 - state[0]) - state[2] + math.pi) % (2 * math.pi)) - math.pi

            # get measurements from landmark to start
            tail = ((math.atan2(0 - state[1], 0 - state[0]) - state[2] + math.pi) % (2 * math.pi)) - math.pi
            back = math.sqrt((state[0] - 0)**2 + (state[1] - 0)**2)

            # control loop for heading to landmark (within 2 inches)
            
            if not there:
                there = navigate_to(0.5, 0, state)
            elif not done:
                done = navigate_to(0, 0.5, state)
            else:
                enc.stop()
            
                

    except KeyboardInterrupt:
        print("Stopped by user")

    finally:
        GPIO.cleanup()
