import numpy as np
import math

def init_particles(n: int):
    # generate 500 particles near origin and equal prob. for each weight
    particles = np.random.randn(n, 3) * 0.05
    weights = np.ones((n, 1)) / n
    return np.hstack([particles, weights])

def predict_particles(particles, dist, delta_theta, dt):
    # add some normally dist. noise to the particles based on where we moved and recalc each point's x, y
    particles[:, 0] = particles[:, 0] + (dist + np.random.randn(len(particles)) * .05) * math.cos(particles[:, 2])
    particles[:, 1] = particles[:, 1] + (dist + np.random.randn(len(particles)) * .05) * math.sin(particles[:, 2])
    
    rotation = (delta_theta / 131) * (math.pi / 180)
    particles[:, 2] = (particles[:, 2] + rotation * dt) + (np.random.randn(len(particles)) * .001)
    return particles


def weight_assignment(particles, scan, landmarks, sigma):
    weights = []
    
    # look at each particle, comparing its proclaimed position to known landmarks 
    for i, p in enumerate(particles):
        best_match = None
        best_diff = float('inf')

        # how far is the particle and how close is its heading?
        for lx, ly in landmarks:
            dist = (math.sqrt((lx - p[0])**2 + (ly - p[1])**2))
            a = math.atan2(ly - p[1], lx - p[0]) - p[2]
            z_pred = np.array([dist, a])

            # look through actual scans being taken
            for s_quality, s_angle, s_distance in scan:
                scan_rads = (s_angle / 180) * math.pi
                diff = scan_rads - a

                # gets absolute difference in heading from landmark and assigns point if distances are within 28mm
                diff = abs((diff + math.pi) % (2 * math.pi) - math.pi)
                if diff < best_diff:

                    # is the distance of particle (from the landmark) within 28mm and 5 degrees. of actual scan
                    if abs((dist * 1000) - s_distance) < 28:
                        if best_diff == float('inf'):
                            best_match = [s_quality, s_angle, s_distance]
                            best_diff = diff
                        else:
                            if abs(best_diff - diff) < 5:
                                best_match = [s_quality, s_angle, s_distance]
                                best_diff = diff

        # if no match found for this particle, skip it
        if best_match is None:
            continue
        
        # if a good candidate is found for this particle, flatten its probability between 0 and 1
        bm_dist = (best_match[2]) / 1000
        bm_angle = (((best_match[1] / 180) * math.pi) + math.pi) % (2 * math.pi) - math.pi
        residual = np.array([bm_dist, bm_angle]) - z_pred
        weight = np.exp(-0.5 * (residual / sigma)**2)
        weights.append(weight)

        # add good weights to a weights array for other calculations
        particles[i, 3] = weight
    return weights

def resample(particles):
    # normalize the weights to between 0 and 1 summing to 1
    weights = (particles[:, 3] / particles[:, 3].sum())

    # get random points clustered around weights (more around higher weights)
    indices = np.random.choice(len(particles), size=len(particles), p=weights)

    # remake the particles with these points and normalize the weights to even prob for next cycle
    particles = particles[indices]
    particles[:, 3] = (1 / len(particles))
    return particles

def estimate(particles):
    weights = particles[:, 3]
    angles = particles[:, 2]
    x_mean = np.sum(particles[:, 0] * weights)
    y_mean = np.sum(particles[:, 1] * weights)
    theta_mean = math.atan2(np.sum(np.sin(angles) * weights), np.sum(np.cos(angles) * weights))

    state = [x_mean, y_mean, theta_mean]
    return state
