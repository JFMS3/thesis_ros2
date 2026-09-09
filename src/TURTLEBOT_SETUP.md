
# TurtleBot 4 Steps
The following steps will configure the TurtleBot to work with ROS2 Jazzy Jalisco. We will be configuring the Turtlebot to operate in **Discovery Server Mode**. Although it has more complex setup than the normal *Simple Discovery* mode, it greatly reduces WiFi congestion and improves stability especially for multi-robot applications.

<img width="600" height="648" alt="PXL_20260807_035233042" src="https://github.com/user-attachments/assets/3956aa6c-e460-419f-9e3a-b1f3949581d7" />

## Updating the Turtlebot's ROS2 firmware.
*With TurtleBot 4 on dock*
1. Download the latest [I.0.0.FastDDS](https://github.com/iRobotEducation/create3_docs/releases/download/I.0.0/Create3-I.0.0.FastDDS.swu) file. Do **not** download the CycloneDDS file.
2. Connect to Turtlebot4 network (password is Turtlebot4) and navigate to http://10.42.0.1:8080
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
    If this doesn't work, just clicking on the link above will download it (3.5GB total)
3. Extract the zip file

## Remove and backup the MicroSD card
1. Open the TurtleBot tray (you will need to disconnect the camera and lidar cables)
2. Carefully extract the microSD card from the Raspberry Pi as shown below.

<img width="300" height="320" alt="PXL_20260807_045517124" src="https://github.com/user-attachments/assets/ac9ae9dc-184c-4259-9a97-e411b4347ca3" />
    
3. In PowerShell, run `Get-Disk`. You should see the MicroSD card show up (roughly 32GB of storage capacity).
4. Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/). We will use this to flash the new Jazzy image.
5. Open the Imager. You may get prompted to install device software, select yes.
6. Select **Raspberry Pi 4** as the device. Click next.
7. Select **Use Custom** as the OS, and select the .img file from the extracted Jazzy zip file (usually the only file in that folder). Click next.
8. Select the microSD card as the Storage. If it is labelled *read-only*, that might mean your SD card adapter is in the *lock* position, so unlock it and try again.
9. Do not customise the image. Click Write and wait for it to finish.
10. Eject the microSD card and reinsert into the TurtleBot.

## Put Raspberry Pi on lab router
1. From a WiFi capable computer, connect to the Turtlebot4 network (password is Turtlebot4).
2. Wait a few minutes. Then, in Ubuntu, run `ssh ubuntu@10.42.0.1`. You may get an error that 'Remote Host Identification has changed'. If so, run the suggested command `ssh-keygen -f ...`.
3. Enter the password *turtlebot4*. Keep checking you are connected to the Turtlebot4 network, errors may cause you to disconnect.
4. Run the command `turtlebot4-setup` and navigate to *WiFi Setup*.
5. Set the following parameters and leave the others as their defaults
    - Wi-Fi Mode: **Client**
    - SSID: *name of your WiFi network*
    - Password: *password of your WiFi network*
    - Band: **Any**
6. Click Save and enter *turtlebot4* as the password when prompted.
7. In the main menu, select *Apply Settings*. The settings should be applied and the connection should be automatically closed. 

## Find the robot's new IP address
1. Connect to your router's network and navigate to its admin page (usually 192.168.1.1). You should see the device listed as *turtlebot4*, and its corresponding IP address (e.g. 192.168.1.148)
2. It is highly recommended to reserve that IP address on the router. For a DD-WRT router, go to Services and add a *Static Lease*
3. Fill using the MAC and IP address of the turtlebot4. The hostname can be any simple identifier, and the lease time can be left blank

## Configure Discovery Server
1. In Ubuntu, run the command `ssh ubuntu@<TB_IP>`, where TB_IP is the IP address of the turtlebot.
2. Once connected, run `sudo apt update` and `sudo apt upgrade`.
3. Run `sudo reboot` and wait a minute.
4. SSH into turtlebot again, and run `turtlebot4-setup`.
5. Navigate to ROS Setup --> Discovery Setup. Configure the following settings
   - Enabled: **True**
   - Onboard Server - Port: **11811**
   - Onboard Server - Server ID: **0**
   - Offboard Server - IP: *blank*
   - Offboard Server - Port: **11811**
   - Offboard Server - Server ID: **1**
6. Save the settings, and apply them in the main menu.
7. Exit *turtlebot4-setup*. Run `turtlebot4-source`, followed by `turtlebot4-daemon-restart`.

## Finish Configuration on Computer
1. Run the following
   ```bash
       wget -q0 - https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/jazzy/turtlebot4_discovery/configure_discovery.sh | bash <(cat) </dev/tty
   ```
2. You will get asked some questions
    - ROS_DOMAIN_ID [0]: *press ENTER*
    - Discovery Server ID: *press ENTER*
    - Discovery Server IP: **TB_IP**
    - Discovery Server Port 1 [11811]: *press ENTER*
    - Re-enter the last server (r), add another server (a), or done (d): **d**
3. Run `source ~/.bashrc`

## Manual Control
The Turtlebot should be configured now, yippee! In a **fresh** Ubuntu terminal, run the following commands to run a manual control package.
```bash
sudo apt update
sudo apt install ros-jazzy-teleop-twist-keyboard
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix teleop_twist_keyboard
```
At the end, you should see `/opt/ros/jazzy`. Then run
```
ros2 pkg executables --full-path teleop_twist_keyboard
```
You should see something like `/opt/ros/jazzy/lib/teleop_twist_keyboard/teleop_twist_keyboard`. Finally,
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## Run a Package
Like other ROS2 packages, you can compile and run it as follows
```
colcon build --symlink-install --packages-select thesis_platform_controller
source install/setup.bash
ros2 run thesis_platform_controller platform_controller
```

You can also run the package with parameters (if you accept parameters in the code) as below
```
ros2 run thesis_platform_controller platform_controller --ros-args -p param_name:=param_value
```