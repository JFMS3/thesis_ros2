# Thesis ROS2 Implementation

## Launch process

Ensure the drone and platform are flat on startup. The software begins by taking the drone's resting quaternion as the level plane.


VENV creation:
(in ~/ros_ws)
python3 -m venv ros_env --system-site-packages
source ros_env/bin/activate