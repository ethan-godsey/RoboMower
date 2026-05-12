import time
import smbus2
import math
import numpy as np
from rplidar import RPLidar
import queue
import threading
import RPi.GPIO as GPIO
import collections
from config import constants as c
from drivers import imu_trial as imu
from drivers import wheel_encoder_trial as enc

# queues to get most recent data from varying feedback speeds
lidar_data = queue.Queue()
imu_data = collections.deque(maxlen=20)

# setup encoder
GPIO.setmode(GPIO.BCM)
enc.setup_enc()

# setup LiDAR
GPIO.setup(c.LIDAR_MOTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(c.LIDAR_MOTOR_PIN, 10000)
pwm.start(100)
lidar = RPLidar(c.LIDAR_PORT)
lidar.reset()
lidar.clean_input()
lidar.start_motor()

# setup IMU
bus = smbus2.SMBus(1)

# initial time to reference
start = time.time()
state = np.array([0.0, 0.0, 0.0])
p = np.zeros((3, 3))

# q is the bias estimate in a constant matrix. it tells us how much we trust each input and can 
# be adaptive in real systems that go on varying terrain
q = np.diag([0.01, 0.01, 0.001])
R = np.diag([0.02, 0.02])
lndmrks = [(1, 1)]
prev_dist = 0.0

'''func: imu_read: get output from IMU and put into queue'''
def imu_read():
    # Activate IMU
    sum_gyro_bias = 0.0
    bus.write_byte_data(c.IMU_ADDR, c.PWR, 0)

    # Gather sample of bias while LiDAR spins up
    for i in range(500):
        gz = imu.read_word(c.GZ)
        print(gz)
        sum_gyro_bias += gz
    
    # average out bias to correct later
    gyro_bias_z = sum_gyro_bias / 500
    print(gyro_bias_z)
    # add corrected head value to queue with x and y from accelerometer
    while True:
        print(imu.read_word(c.GZ))
        unbiased_gz = imu.read_word(c.GZ) - gyro_bias_z
        imu_data.append((unbiased_gz, imu.read_word(c.AX), imu.read_word(c.AY)))

'''func: encoder_read: get output of encoder based on when event being triggered'''
def encoder_read():
    GPIO.add_event_detect(c.A_C1, GPIO.RISING, callback=enc.encoder_a_callback)
    GPIO.add_event_detect(c.B_C1, GPIO.RISING, callback=enc.encoder_b_callback)

    while True:
        time.sleep(0.001)

'''func: lidar_read: get output of lidar spin, and then put in queue'''
def lidar_read():    
    while True:
        try:
            for scan in lidar.iter_scans():
                lidar_data.put(scan)
        except:
            lidar.clean_input()
            time.sleep(0.01)

'''func: predict: EKF prediction filter from IMU and encoders
param: state: 
param: P: The uncertainty matrix and how much to trust each sensor
param: d: distance traveled in the last increment
param: delta_theta: change in direction (heading)'''
def predict(state, p, d, delta_theta):

    # calculate time since last step
    global start
    now = time.time()
    dt = now - start
    start = now

    # get new change in x, y positions from last point
    x_new = state[0] + d * math.cos(state[2])
    y_new = state[1] + d * math.sin(state[2])  

    # calculate heading (rotation) about the z-axis we started at
    new_gz = (delta_theta / 131) * (math.pi / 180)
    head_new = state[2] +  new_gz * dt

    # update the state of position for next func call
    state = np.array([x_new, y_new, head_new])

    # F will be the Jacobian matrix (partial derivatives of x/y/head_new)
    F = np.array([[1, 0, (-1 * d) * math.sin(head_new)], [0, 1, d * math.cos(head_new)], [0, 0, 1]])
    
    # P matrix = FPF^T where P represents sensor noise and Q represents tuned error matrix
    p_new = F @ p @ (F.T) + q
    p = p_new

    return state, p_new

def update(state, p, scan, landmarks, R):
    x = state[0]
    y= state[1]
    for (lx, ly) in landmarks:
        dist = (math.sqrt((lx - x)**2 + (ly - y)**2))
        a = math.atan2(ly - y, lx - x) - state[2]
        z_pred = np.array([dist, a])
        H = np.array([[((x - lx) / dist), ((ly - y) / dist), 0], [((ly - y)/(dist * dist)), ((x - lx)/(dist * dist)), -1]])
        best_match = None
        best_diff = float('inf')

        # get the most related point in the scan to our estimate
        for point in scan:
            scan_rads = (point[1] / 180) * math.pi
            diff = scan_rads - a
            diff = abs((diff + math.pi) % (2 * math.pi) - math.pi)
            if diff < best_diff:
                if abs((dist * 1000) - point[2]) < 28:
                    best_match = point
                    best_diff = diff
        
        if best_match is None:
            continue

        # create distance angle pair for best point match of scan
        bm_dist = (best_match[2]) / 1000
        bm_angle = (((best_match[1] / 180) * math.pi) + math.pi) % (2 * math.pi) - math.pi
        z = np.array([bm_dist, bm_angle])
        
        # innovation covariance - uncertainty in the whole system
        S = H @ p @ H.T + R

        # Kalman gain - trust in measurement vs prediction
        s_inv = np.linalg.inv(S)
        K = p @ H.T @ s_inv

        # state correction
        residual = z - z_pred
        residual[1] = (residual[1] + math.pi) % (2 * math.pi) - math.pi
        state = state + K @ (z - z_pred)

        # covariance update - adjust uncertainty for next cycle
        I = np.identity(3)
        p = (I - K @ H) @ p
    return state, p

def move_fwd():
    enc.forward(100)
    time.sleep(3)
    enc.stop()

try:
    delta_thet = 0.0
    imu_thread = threading.Thread(target=imu_read)
    lidar_thread = threading.Thread(target=lidar_read)
    encoder_thread = threading.Thread(target=encoder_read)
    imu_thread.start()
    lidar_thread.start()
    encoder_thread.start()
    time.sleep(5)
    there = False
    count = 0
    while True:
        
        # get imu header data from queue and clear old entries (get pops)
        latest_imu = None
        rec_scan = None
        imu_sum = 0
        for meas in range(20):
            try:
                imu_sum += imu_data.popleft()
            except:
                break
        delta_thet = imu_sum / 20

        latest_lidar = None
        while not lidar_data.empty():
            latest_lidar = lidar_data.get()
        if latest_lidar:
            rec_scan = latest_lidar
        
        # get encoder distance
        current_dist = (enc.dist_a + enc.dist_b) / 2
        dist = current_dist - prev_dist
        prev_dist = current_dist

        # get new predicted position and uncertainty matrix and view state
        state, p = predict(state, p, dist, delta_thet) 

        if latest_lidar:
            state, p = update(state, p, rec_scan, lndmrks, R)
        
        #print(state)
        count += 1
        
        distance = math.sqrt((state[0] - 1)**2 + (state[1] - 1)**2)
        head = ((math.atan2(1 - state[1], 1 - state[0]) - state[2] + math.pi) % (2 * math.pi)) - math.pi
        tail = ((math.atan2(0 - state[1], 0 - state[0]) - state[2] + math.pi) % (2 * math.pi)) - math.pi
        back = math.sqrt((state[0] - 0)**2 + (state[1] - 0)**2)
        #print(f"{distance}, {head}")
        if distance > 0.05 and not there:
            if head < -0.05:
                print(f"head: {head}")
                #enc.right_forward(50)
                #enc.left_backward(50)
            elif head > 0.05:
                print(f"head: {head}")
                #enc.left_forward(50)
                #enc.right_backward(50)
            else:
                print(f"head: {head}") 
                #enc.forward(100)
        elif distance < 0.05 and not there:
            there = True
        elif back > 0.05 and there:
            if tail < -0.05:
                print(tail)
                #enc.left_backward(50)
                #enc.right_forward(50)
            elif tail > 0.05:
                print(tail)
                #enc.right_backward(50)
                #enc.leftt_forward(50)
            else:
                print(tail)
                #enc.forward(100)
        else:
            enc.stop()
        
        if count % 10 == 0:
            print(f"state: {state}, dist: {dist}, delta_theta: {delta_thet}")
    
            

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    GPIO.cleanup()
