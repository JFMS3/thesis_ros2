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

### Additional Motive Settings
Without proper calibration, rigid body detection can be quite jittery, as the software mistakes random reflections around the room for markers.
From experimentation, it was found the following camera settings worked well.
* Exposure: 500us
* Threshold: 200
* Gain: 4 (mid-range)

## Windows Settings
Motive runs on Windows, but ROS2 runs in WSL. Essentially WSL acts as a small Linux machine running inside Windows, which by default is walled off in its own private network. We have to get rid of that 'wall' so streaming can occur.

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

The necessary code for reading this data can be found in *optitrack_bridge_node.py* file.

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