# VSLAM - WashU Robotics
## Integration of Visual Slam with ArduCopter for Quadrotor Perception and Navigation
##### A Guide for Integrating ArduCopter with RTAB-Map and ROS2 Using Intel Realsense and NVIDIA Jetson Orin Nano
By Chris Brusie - WashU Robotics

## Hardware Requirements
- NVIDIA Jetson Orin Nano Developer Kit (8GB)
- Intel Realsense Depth Camera D435i/D435if
- H7-based flight controller 
    - MATEK H743 Slim V3 is used in this example

## Prerequisites
- Flash NVIDIA Jetpack 6.2 (R36 Rev4.3)
    - NVME SSD highly recommended over SD card
- Install ROS2 Humble Desktop and configure the environment
    - workspace named `ros2_ws` in this example
## Hardware Setup
| Flight Controller | Jetson |
|----------|----------|
| RX7   | UART1_TX (pin 8)   |
| TX7   | UART1_RX (pin 10)   |
| RTS7   | UART1_RTS (pin 11)   |
| CTS7   | UART1_CTS (pin 36)  |
| GND   | GND (pin 6)   |
- In addition, I've connected the Jetson and flight controller via USB. 

## 1. Realsense SDK2.0 and ROS Wrapper
#### Build RealSense SDK2.0 for JP6.2
> Note: Realsense SDK2.0 does not officially support Jetpack 6.2. Below is a workaround.
 Credit goes to @GoPro1147 from [this issue](https://github.com/IntelRealSense/librealsense/issues/13713) for the following patch. 
- Clone the development branch of Librealsense
- Replace `scripts/patch-realsense-ubuntu-L4T.sh` with the modified version in this repo. (Credit to GoPro1147)
- Replace `source_sync.sh` with the latest version from NVIDIA. It can be downloaded from [here](https://docs.nvidia.com/jetson/archives/r36.4.3/DeveloperGuide/IN/QuickStart.html#to-flash-the-jetson-developer-kit-operating-software). 
    - the file is located in the `Linux_For_Tegra/source` folder. 
- Now, follow [Intel's guide](https://github.com/IntelRealSense/librealsense/blob/development/doc/installation_jetson.md) guide to build Librealsense from source using the native backend
    - this will only work if you build from source using the **native backend**, not RSUSB backend. 
- Cmake build fail: no Cmake Cuda Compiler found
```sh
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```
- Cmake build fail: missing OpenGL packages
```sh
sudo apt update
sudo apt install libglu1-mesa-dev freeglut3-dev mesa-common-dev
```
#### Install RealSense-ROS Wrapper
- Follow [this guide](https://github.com/IntelRealSense/realsense-ros) to install the RealSense ROS Wrapper **from source**
- To test, launch camera node:
```sh
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```
- Open RVIZ:
```sh
rviz2
```
- Visalize depth cloud topics. 
## 2. RTABMAP_ROS
#### Install RTAB-Map-ROS
- Follow [this guide](https://github.com/introlab/rtabmap_ros/tree/ros2?tab=readme-ov-file#rtabmap_ros) to install RTAB-Map ROS **from source**. 
- Colcon build fails on realsense2_camera_msgs package
```sh
rm -rf ~/ros2_ws/build/realsense2_camera_msgs/ament_cmake_python/realsense2_camera_msgs
```
- Build again
The system will run out of memory, so we must increase swap size and likely build packages indivisually. 
```sh
sudo swapoff -a
sudo dd if=/dev/zero of=/swapfile bs=1G count=8
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```
- Once all packages build, test SLAM with any of these launch files:
```sh
ros2 launch realsense_d435i_stereo.launch.py
```
```sh
ros2 launch realsense_d435i_color.launch.py
```
```sh
ros2 launch realsense_d435i_infra.launch.py
```
 - Install Octomap
```sh
sudo apt-get install ros-humble-octomap*
```
 - **(optional)** Turn fan on cool mode (it was getting kinda hot)
    - Open `/etc/nvfancontrol.conf` with any editor. 
    - Find `FAN_DEFAULT_PROFILE` (near bottom), change from `quiet` to `cool`
    - Change takes effect on next reboot

## 3. ArduPilot DDS Setup
#### Set up the ArduPilot Build Environment
- If not already done, install Git:
```sh
sudo apt-get update
sudo apt-get install git
```
- Clone the main ArduPilot Repo
```sh
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git && cd ardupilot
```
- This script installs all required packages automatically:
```sh
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
- Reload
```sh
. ~/.profile
sudo reboot
```
#### ArduPilot ROS Repos
```sh
cd ros2_ws/src
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos
vcs import --recursive < ros2.repos 
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --rosdistro humble --from-paths src --ignore-src
```
#### Micro-XRCE-DDS-Gen Build Dependency (used for the flight controller to communicate with DDS)
```sh
sudo apt install default-jre -y
cd ros2_ws
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen/
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
sudo reboot
```
- Rebuild the workspace
```sh
cd ros2_ws
colcon build --packages-up-to ardupilot_dds_tests 
```
#### Modifying the DDS Library in ArduPilot Source Code
> Note:  By default, ArduPilot’s DDS library enables many things that we will not use. Since we are using a serial connection to the Jetson with baudrate of 2,000,000 (slower compared to other options like ethernet), trying to pass all this data freezes the connection. Apparently it can go faster, but I couldn’t get that to work. For now, we just disable some things that will not be used. 

- Use your editor of choice to open `~/ardupilot/libraries/AP_DDS/AP_DDS_config.h`
- Find `AP_DDS_EXPERIMENTAL_ENABLED` (line 22) and disable it by changing the 1 to 0. 
#### Build ArduCopter firmware with DDS enabled and upload to the flight controller
```sh
cd ~/ardupilot
./waf configure --board MatekH743-bdshot --enable-dds 
./waf copter --upload
```
- replace `MatekH743-bdshot` with your board name. 


## 4. ArduPilot Communication Parameters
> Note: ArduPilot parameters can be set using MAVProxy on the Jetson, or using Mission Planner on a Windows machine. This example will use Mission Planner, but either method will work. 

#### Serial Port Parameters
```
BRD_SER1_RTSCTS, 1
SERIAL1_BAUD, 2000 
SERIAL1_PROTOCOL, 45
```
#### DDS Parameters
```
DDS_DOMAIN_ID, 0
DDS_ENABLE, 1
```
#### Test the Connection
- Run MAVProxy over USB. Change the output address to your laptop's IP address.
```sh
mavproxy.py --master=/dev/ttyACM* --out=udp:172.27.51.230:14550
```
- Run the Micro-ROS agent
```sh
cd ~/ros2_ws
ros2 run micro_ros_agent micro_ros_agent serial -b 2000000 -D /dev/ttyTHS1
```
You should now be able to see topics pusblished by the /ap node. 
Verify that the connection is stable by viewing the contents of a topic:
```sh
ros2 topic echo /ap/time
```
## 5. Odometry Setup
This section is about taking the dynamic transformation from `odom` frame to `camera_link` frame provided by RTAB-Map odometry, and computing the transformation from `odom` frame to `base_link` frame.
- `transform_pkg` includes a node which computes a static transformation between `base_link` and `camera_link`. 
    - This transformation is the spatial relationship between the camera and flight controller on the quadcopter. 
- `realsense_d435i_stereo_real.launch.py` is slightly modified from a previous RTAB-Map example, but it runs the statis transform node and passes `base_link` frame ID instead of `camera_link`. 
#### Build and Run
- build `transform_pkg`
```sh
cd ~/ros2_ws
colcon build --packages-selet --symlink-install transform_pkg
```
- launch:
```sh
ros2 launch realsense_d435i_stereo_real.launch.py
```
- The transform is being published on `/tf`, so we need to relay it to `/ap/tf`.
```sh
sudo apt install ros-humble-topic-tools
ros2 run topic_tools relay /tf /ap/tf
```
#### More ArduPilot Parameters
These parameters configure the Kalman Filter to use VIO. 


```
AHRS_EKF_TYPE, 3 (use EKF3 kalman filter)
EK3_ENABLE, 1 
EK3_SRC1_POSXY, 6 (set horizontal position source to externalNAV)
EK3_SRC1_POSZ, 1 (set vertical position source to barometer)
EK3_SRC1_VELXY, 6 (set horizontal velocity source to externalNAV)
EK3_SRC1_VELZ, 6 (set vertical velocity source to externalNAV)
EK3_SRC1_YAW, 6 (set yaw source to externalNAV)
VISO_TYPE, 1 (enable visual odometry)
ARMING_CHECK, 388598 (make sure visOdom health is checked)
```

## 6. Run
- launch MAVProxy over USB (set the destination IP to your Windows PC address)
```sh
mavproxy.py --master=/dev/ttyACM* --out=udp:172.27.51.230:14550
```
- launch RTAB-Map
```sh
cd ~/ros2_ws/src/rtabmap_ros/rtabmap_examples/launch
ros2 launch realsense_d435i_stereo_real.launch.py
```
- launch the Micro-ROS agent over serial
```sh
cd ~/ros2_ws
ros2 run micro_ros_agent micro_ros_agent serial -b 2000000 -D /dev/ttyTHS1
```
- relay `/tf` to `/ap/tf`
```sh
ros2 run topic_tools relay /tf /ap/tf
```

