# VSLAM - WashU Robotics
## Integration of Visual Slam with ArduPilot for Quadrotor Perception and Navigation
##### A Guide for Integrating ArduPilot with RTAB-Map and ROS2 Using Intel Realsense and NVIDIA Jetson Orin Nano
By WashU Robotics

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


## Realsense SDK2.0 and ROS Wrapper
##### Build RealSense SDK2.0 for JP6.2
- Follow this guide
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
##### Install RealSense-ROS Wrapper
- To test, launch camera node:
```sh
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```
- Open RVIZ:
```sh
rviz2
```
- Visalize depth cloud topics. 
## RTABMAP_ROS
##### Install RTAB-Map-ROS
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

## ArduPilot DDS Setup


