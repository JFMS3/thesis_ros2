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


## Acados and Casadi installation
_Casadi_ provides a framework for symbollically defining system dynamics and cost equations.
_Acados_ provides the solvers.

The first step is building the Acados C libraries
```shell
cd ~/
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j$(nproc)
```

Then get the t_renderer binary. Acadoes uses Tera to render C code templates.
```shell
mkdir -p ~/acados/bin
cd ~/acados/bin
wget https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64
mv t_renderer-v0.2.0-linux-amd64 t_renderer
chmod +x t_renderer
```

Then set environment variables. Note that ros2_run needs these vars set in the same shell it is launched from. Best to set these vars explicitly in the launch file (i.e. thesis_launch) or make sure the launch env inherits .bashrc
```shell
echo 'export ACADOS_SOURCE_DIR=$HOME/acados' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib' >> ~/.bashrc
source ~/.bashrc
```

Then pip install the Python interfaces
```shell
pip install -e interfaces/acados_template --break-system-packages
pip install casadi --break-system-packages
```

To verify it works run this
```shell
cd ~/acados/examples/acados_python/getting_started
python3 minimal_example_ocp.py
```