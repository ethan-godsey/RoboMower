import time
import math
import numpy as np

# q is the bias estimate in a constant matrix. it tells us how much we trust each input and can 
# be adaptive in real systems that go on varying terrain
q = np.diag([0.01, 0.01, 0.001])
R = np.diag([0.02, 0.02])
start = time.time()

'''func: predict: EKF prediction filter from IMU and encoders
param: state: 
param: P: The uncertainty matrix and how much to trust each sensor
param: d: distance traveled in the last increment
param: delta_theta: change in direction (heading)'''
def predict(state, p, d, delta_theta):

    # calculate time since last step
    now = time.time()
    dt = now - start
    global start
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

def update(state, p, scan, landmarks):
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
        
        # dont accept super inconsistent corrections
        if abs(residual[1]) > (math.pi / 4):
            continue

        '''µt = µt + K (zt − h(µt))'''
        state = state + K @ (residual)

        '''Σt = (I − Kt * Ht)Σt'''
        # covariance update - adjust uncertainty for next cycle
        I = np.identity(3)
        p = (I - K @ H) @ p

    '''return µt,Σt'''
    return state, p