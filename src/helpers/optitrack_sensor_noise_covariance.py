import numpy as np
from pathlib import Path
import re
import os
from matplotlib import pyplot as plt

current_dir = os.path.dirname(os.path.abspath(__file__))
logpath = os.path.abspath(os.path.join(current_dir, '..', "..", "log", "quadcopter_position_stationary_logging.txt"))

pattern = r"^([+-]?\d+\.\d+), ([+-]?\d+\.\d+), ([+-]?\d+\.\d+)$"
position_list = []

with open(logpath, 'r') as file:
    matched_groups = re.findall(pattern, file.read(), re.MULTILINE)
    all_positions = [[float(num) for num in group] for group in matched_groups]


position_data = np.array(all_positions)
cov_matrix = np.cov(position_data, rowvar=False)
cov_xy = cov_matrix[0, 1]
cov_xz = cov_matrix[0, 2]
cov_yz = cov_matrix[1, 2]

print(cov_matrix)

eigvals, eigvecs = np.linalg.eigh(cov_matrix)
print(eigvals, eigvals.max() / eigvals.min())

plt.figure()
plt.plot(position_data[:, 0], label='x')
plt.plot(position_data[:, 1], label='y')
plt.plot(position_data[:, 2], label='z')
plt.legend()
plt.savefig("quad_position_trace")
plt.close()
z = (position_data - position_data.mean(0)) / position_data.std(0)
print(np.abs(z).max())


'''
Platform covariance
[[ 8.22292703e-07 -1.25930039e-07 -3.41836164e-08]
 [-1.25930039e-07  1.06470785e-07  3.35939481e-08]
 [-3.41836164e-08  3.35939481e-08  1.71409207e-07]]
'''

'''
Quadcopter position covariance
[[4.05334208e-09 4.08163319e-09 5.91631335e-09]
 [4.08163319e-09 5.55542538e-09 5.95189493e-09]
 [5.91631335e-09 5.95189493e-09 8.73493455e-09]]
'''

'''
Quadcopter attitude covariance
[[ 1.93045004e-06 -8.30303711e-06]
 [-8.30303711e-06  5.02677355e-05]]
'''
