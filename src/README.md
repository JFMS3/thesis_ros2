# Thesis ROS2 Implementation

## Launch process

Ensure the drone and platform are flat on startup. The software begins by taking the drone's resting quaternion as the level plane.


VENV creation:
(in ~/ros_ws)
python3 -m venv ros_env --system-site-packages
source ros_env/bin/activate

(in each terminal cd to ~/ros_ws and do source install/setup.bash)
ros2 run thesis_mpc_controller kalman_filter_node --ros-args -p R_pos_diag:="[1.0e-5, 1.0e-5, 1.0e-5]" -p sigma_accel:="[0.5, 0.5, 0.3]" -p nis_threshold:=11.34
ros2 run thesis_mpc_controller synthetic_source
ros2 bag record /full_quadcopter_state /observed_nis -o nis_test