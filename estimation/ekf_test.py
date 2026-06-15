import ekf as ekf
import numpy as np

state = [0, 0, 0]
p = np.zeros((3, 3))
landmarx = [(0.5, 0)]
scans = [[12, 0.35, 470]]
new_state, new_p = ekf.predict(state, p, 0.01, 0.1)
ekf.update(new_state, new_p, scans, landmarx)
