import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'estimation'))
import particle_filter as pf



particles = pf.init_particles(5)
fake_scan = [[10, 5.0, 950], [10, 180.0, 1100], [10, 2.0, 980]]
fake_landmarks = [(1, 0)]
print(particles)
pf.weight_assignment(particles, fake_scan, fake_landmarks, sigma=0.5)
print(particles[:, 3])  # do the weights change?
