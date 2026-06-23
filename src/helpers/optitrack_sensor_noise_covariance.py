import numpy as np
from pathlib import Path
import re

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
print(position_variance)
print(rotation_variance)