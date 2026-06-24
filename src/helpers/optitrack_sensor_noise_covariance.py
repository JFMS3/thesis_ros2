import numpy as np
from pathlib import Path
import re
from thesis_optitrack_bridge.frame_transformer import FrameTransformer


logpath = Path.home() / ".ros" / "log" / "python3_8571_1782280280689.log"
pattern = r"position \((.*?)\) and rotation \((.*?)\)"
position_list = []
rotation_list = []
euler_list = []
transformer = FrameTransformer()

with open(logpath, 'r') as file:
    for line in file:
        match = re.search(pattern, line.strip())

        if match:
            position_string, rotation_string = match.groups()
            position = list(map(float, position_string.split(", ")))
            rotation = list(map(float, rotation_string.split(", ")))

            q = transformer.normalise_quat(rotation)
            phi, theta, _ = transformer.quat_to_euler(q)
            position_list.append(position)
            rotation_list.append(rotation)
            euler_list.append([phi, theta])


position_data = np.array(position_list)
rotation_data = np.array(rotation_list)
euler_data = np.array(euler_list)

position_variance = np.var(position_data, axis=0, ddof=1)
rotation_variance = np.var(rotation_data, axis=0, ddof=1)
euler_variance = np.var(euler_data, axis=0, ddof=1)

print(position_variance)
print(rotation_variance)
print(euler_variance)