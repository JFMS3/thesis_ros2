# TurtleBot 4 Steps
The following steps assume use of the TurtleBot with ROS2 Jazzy Jalisco. We will be configuring the Turtlebot to operate in **Discovery Server Mode**. Although it has more complex setup than the normal *Simple Discovery* mode, it greatly reduces WiFi congestion and improves stability especially for multi-robot applications.

## Updating the Turtlebot's ROS2 firmware.
*With TurtleBot 4 on dock*
1. Download the latest [I.0.0.FastDDS](https://github.com/iRobotEducation/create3_docs/releases/download/I.0.0/Create3-I.0.0.FastDDS.swu) file. Do **not** download the CycloneDDS file.
2. Connect to TurtleBot4 network and navigate to http://10.42.0.1:8080
3. Select *Update* from top ribbon and upload FastDDS file. You should here its chime when uploading.

## Installing ROS2 Jazzy on your computer
1. Reconnect to your normal Internet. Ensure Ubuntu 24.04 is installed.
2. On Ubuntu, install [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) if you haven't already.
3. Run the following commands. If ROS2 Jazzy was installed correctly, you should see 'jazzy' as the output.
    ```bash
    source /opt/ros/jazzy/setup.bash
    echo "$ROS_DISTRO"
    ```
4. Install the TurtleBot4 desktop package:
    ```
    sudo apt update
    sudo apt install ros-jazzy-turtlebot4-desktop
    ```
5. If installed correctly, you should see turtlebot4 packages when running `ros2 pkg list | grep turtlebot4`.

## Reset the Create3 Config
This step removes the old ROS2 packages (not the newly installed Jazzy ones)
1. Hold the left and right buttons (1 and 2) on the Turtlebot until you see a blue light and hear a sound.
2. Connect to the robot's unique **Create-XXXX** network. Record this name. For instance robot 1 has network  *Create-05FC*.
3. Navigate to `http://192.168.10.1/home` and go to the *About* page in the top ribbon. It may take a few minutes to load, even if it initially cannot find this address.
4. Scroll to the bottom and click *Factory Reset*. It is finished when you hear its chime.

## Download the Jazzy Lite image
At this point, you should no longer be able to see that Create-XXXX network.
1. Reconnect to your regular Internet. 
2. On **powershell** run the following
    ```bash
    cd ~/Downloads
    wget https://download.ros.org/downloads/turtlebot4/turtlebot4_jazzy_lite_2.0.2.zip
    ```

## Remove and backup the MicroSD card
1. 