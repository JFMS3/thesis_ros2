# Streaming Motive Data to ROS2

(Note the legacy 2.1.1 version of Motive was used for this thesis)

## Motive Settings
Open the Data Streaming panel in Motive, and configure it to transmit data to its local interface. Ensure that data is being streamed via *Unicast* not multicast. The following settings were used (enable advanced settings in the panel)
* Local Interface: 192.168.4.105
* Rigid Bodies: Enabled
* Transmission Type: Unicast
* Command Port: 1510
* Data Port: 1511
* Multicast Interface: 239.255.42.99
* Broadcast VRPN Data: Disabled

Assuming a rigid body is in frame, Motive should now be transmitting data continuously. You can verify this by downloading **NatNet 4.x.x** and opening the SampleClient.exe executable in /Samples/bin/.

## Windows Settings
Motive runs on Windows, but ROS2 runs in WSL. Essentially we have a small linux machine running inside Windows, which by default is walled off in its own private network. We have to tear that wall down.

Simply add the following lines in .wslconfig, located in the root user directory (i.e. C:\Users\User\.wslconfig).

```
[wsl2]
networkingMode=mirrored

[experimental]
hostAddressLoopback=true
```

Then restart WSL.

## Python Subscriber
To finally see data, we need a package that subscribes to the Optitrack topic and prints the output.

Firstly, copy the following files from NatNet 4.x.x/NatNetSDK/Samples/PythonClient into your Optitrack bridge directory:
* DataDescriptions.py
* NatNetClient.py
* MoCapData.py

Moreover, ensure you have a mocap.yaml file located at the base level of this directory.

The necessary code for reading this data can be found in this directory's optitrack_bridge_node.py file, namely

```python
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Quaternion
from thesis_interfaces.msg import QuadcopterState, PlatformState
from .frame_transformer import FrameTransformer
from NatNetClient import NatNetClient

class OptitrackBridgeNode(Node):
    def __init__(self):
        super().__init__('optitrack_bridge')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.declare_parameter('drone_body_id', 1)
        self.declare_parameter('platform_body_id', 2)
        self.drone_id = self.get_parameter('drone_body_id').value
        self.platform_id = self.get_parameter('platform_body_id').value

        self.declare_parameter('server_address', '192.168.4.105')
        self.declare_parameter('client_address', '192.168.4.105')
        self.declare_parameter('use_multicast', False)
        self.server_address = self.get_parameter('server_address').value
        self.client_address = self.get_parameter('client_address').value
        self.use_multicast = self.get_parameter('use_multicast').value

        self.drone_publisher = self.create_publisher(QuadcopterState, 'quadcopter_state', 10)
        self.platform_publisher = self.create_publisher(PlatformState, 'platform_state', 10)

        self.transformer = FrameTransformer()
        self.get_logger().info("Optitrack bridge node has begun!")

        self.client = NatNetClient()
        self.client.set_server_address(self.server_address)
        self.client.set_client_address(self.client_address)
        self.client.set_use_multicast(self.use_multicast)
        self.client.rigid_body_listener = self.rigid_body_callback

        mode = 'multicast' if self.use_multicast else 'unicast'
        self.get_logger().info(f"Connecting to Motive at {self.server_address} ({mode})")
        if not self.client.run("d"):
            self.get_logger().error("NatNet Client failed to start")
            return

        self.get_logger().info("Optitrack bridge has started!")

    ...
```

Then run the following commands to build your package and (hopefully) start seeing some data.
```
colcon build --symlink-install --packages-select thesis_optitrack_bridge
source install/setup.bash
ros2 run thesis_optitrack_bridge optitrack_bridge
```

For logging a run:
```
ros2 bag record -o ~/thesis_ros2/bags/run_$(date +%Y%m%d_%H%M%S) /quadcopter_state
```