import numpy as np
from pathlib import Path
import re
import matplotlib.pyplot as plt

logpath = Path.home() / ".ros" / "log" / "python3_18481_1782196456788.log"
pattern = r"position \((.*?)\) and rotation \((.*?)\)"
position_list = []
rotation_list = []

with open(logpath, 'r') as file:
    for line in file:
        match = re.search(pattern, line.strip())

        if match:
            position_string, rotation_string = match.groups()
            position = list(map(float, position_string.split(", ")))
            rotation = list(map(float, rotation_string.split(", ")))
            position_list.append(position)
            rotation_list.append(rotation)

position_data = np.array(position_list)
rotation_data = np.array(rotation_list)
position_variance = np.var(position_data, axis=0, ddof=1)
rotation_variance = np.var(rotation_data, axis=0, ddof=1)


fig, ax = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
labels = ["qx", "qy", "qz", "qw"]
print(rotation_data)
for i in range(4):
    ax[i].plot(rotation_data[:, i])
    ax[i].set_ylabel(labels[i])
    ax[i].grid(True)

ax[-1].set_xlabel("Sample")

plt.tight_layout()
plt.show()

def quat_to_euler(q):
    x, y, z, w = q

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

euler_data = np.array([quat_to_euler(q) for q in rotation_data])
euler_deg = np.rad2deg(euler_data)
fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

labels = ["Roll (deg)", "Pitch (deg)", "Yaw (deg)"]

for i in range(3):
    ax[i].plot(euler_deg[:, i])
    ax[i].set_ylabel(labels[i])
    ax[i].grid(True)

ax[-1].set_xlabel("Sample")

plt.tight_layout()
plt.show()