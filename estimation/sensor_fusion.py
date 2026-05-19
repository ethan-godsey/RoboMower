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
from config import constants as c
from drivers import imu_trial as imu
from drivers import wheel_encoder_trial as enc

# queues to get most recent data from varying feedback speeds
lidar_data = multiprocessing.Queue()
imu_data = collections.deque(maxlen=20)

# setup encoder
GPIO.setmode(GPIO.BCM)
enc.setup_enc()

# setup LiDAR
GPIO.setup(c.LIDAR_MOTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(c.LIDAR_MOTOR_PIN, 10000)
pwm.start(50)
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
    while True:
        try:
            for scan in lidar.iter_scans(min_len=5):
                queue.put(scan)        
        except:
            lidar.clean_input()

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

    '''µt = g(ut,µt−1)'''
    # get new change in x, y positions from last point
    x_new = state[0] + d * math.cos(state[2])
    y_new = state[1] + d * math.sin(state[2])  

    # calculate heading (rotation) about the z-axis we started at
    new_gz = (delta_theta / 131) * (math.pi / 180)
    head_new = state[2] +  new_gz * dt

    # update the state of position for next func call
    state = np.array([x_new, y_new, head_new])

    '''Σt = Ft pt−1 Ft.T + q'''
    # F will be the Jacobian matrix (partial derivatives of x/y/head_new)
    F = np.array([[1, 0, (-1 * d) * math.sin(head_new)], [0, 1, d * math.cos(head_new)], [0, 0, 1]])
    
    # P matrix = FPF^T where P represents sensor noise (covariance) and Q represents tuned randomness in state transition
    p_new = F @ p @ (F.T) + q
    p = p_new

    return state, p_new

def update(state, p, scan, landmarks, R):
    x = state[0]
    y= state[1]
    for (lx, ly) in landmarks:

        # linearize non-linear distance and angle
        dist = (math.sqrt((lx - x)**2 + (ly - y)**2))
        a = math.atan2(ly - y, lx - x) - state[2]

        # equivalent to h(state)
        z_pred = np.array([dist, a])
        # print(z_pred)
        # calculate jacobian for how much error in state transition causes errors in system
        H = np.array([[((x - lx) / dist), ((ly - y) / dist), 0], [((ly - y)/(dist * dist)), ((x - lx)/(dist * dist)), -1]])

        # get the most related point in the scan to our estimate
        best_match = None
        best_diff = float('inf')
        for point in scan:
            # convert to radians and put in frame of ref to IMU reading, not Lidar heading
            scan_rads = (point[1] / 180) * math.pi
            diff = scan_rads - a

            # gets absolute difference in heading from landmark and assigns point if distances are within 28mm
            diff = abs((diff + math.pi) % (2 * math.pi) - math.pi)
            if diff < best_diff:
                if abs((dist * 1000) - point[2]) < 28:
                    if best_diff == float('inf'):
                        best_match = point
                        best_diff = diff
                    else:
                        if abs(best_diff - diff) < 5:
                            best_match = point
                            best_diff = diff
            # print(best_diff - diff)
        # skip if no match found
        if best_match is None:
            continue
        else:
            print(best_match)
        # create distance angle pair for best point match of scan
        bm_dist = (best_match[2]) / 1000
        bm_angle = (((best_match[1] / 180) * math.pi) + math.pi) % (2 * math.pi) - math.pi
        z = np.array([bm_dist, bm_angle])
        
        # innovation covariance - uncertainty in the whole system
        S = H @ p @ H.T + R

        '''K = Σt * Ht.T (Ht*Σt*Ht.T + Qt)^−1'''
        # Kalman gain: trust in measurement (u) vs prediction (bel(µ))
        s_inv = np.linalg.inv(S)
        K = p @ H.T @ s_inv

        # state correction, measures how wrong predict step was
        residual = z - z_pred
        residual[1] = (residual[1] + math.pi) % (2 * math.pi) - math.pi

        '''µt = µt + K (zt − h(µt))'''
        state = state + K @ (residual)

        '''Σt = (I − Kt * Ht)Σt'''
        # covariance update - adjust uncertainty for next cycle
        I = np.identity(3)
        p = (I - K @ H) @ p

    '''return µt,Σt'''
    return state, p

def move_fwd():
    enc.forward(100)
    time.sleep(3)
    enc.stop()

'''Control loop that goes to a landmark given state'''
def navigate_to(tx, ty, state):
    # calculate distance in 2D space to landmark
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
    if abs(heading_err) < 0.05:
        enc.forward(base_speed)
    else:
        enc.drive(right_speed, left_speed)
    return False


try:
    delta_thet = 0.0
    imu_thread = threading.Thread(target=imu_read)
    lidar_thread = threading.Thread(target=lidar_read)
    encoder_thread = threading.Thread(target=encoder_read)
    lidar_reader = multiprocessing.Process(target=lidar_read, args=(lidar_data,))
    imu_thread.start()
    lidar_thread.start()
    lidar_reader.start()
    encoder_thread.start()
    time.sleep(5)
    there = False
    done = False

    while True:
        
        # get imu header data to make moving average of noise
        latest_imu = None
        rec_scan = None
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
        state, p = predict(state, p, dist, delta_thet) 

        if recent_scan:
            state, p = update(state, p, recent_scan, lndmrks, R)
            
        # print(state)

        # get measurements from start to landmark
        distance = math.sqrt((state[0] - 1)**2 + (state[1] - 1)**2)
        head = ((math.atan2(1 - state[1], 1 - state[0]) - state[2] + math.pi) % (2 * math.pi)) - math.pi

        # get measurements from landmark to start
        tail = ((math.atan2(0 - state[1], 0 - state[0]) - state[2] + math.pi) % (2 * math.pi)) - math.pi
        back = math.sqrt((state[0] - 0)**2 + (state[1] - 0)**2)

        # control loop for heading to landmark (within 2 inches)
        '''
        if not there:
            there = navigate_to(0.5, 0, state)
        elif not done:
            done = navigate_to(0, 0.5, state)
        else:
            enc.stop()
        '''
            

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    GPIO.cleanup()
