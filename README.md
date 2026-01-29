# JawPixARM 6-DOF Robotic Arm (ROS 2 Jazzy)

This repository contains the ROS 2 workspace packages for the **JawPixARM**, a 6-DoF robotic manipulator powered by Dynamixel XL430-w250-t servos and controll through a custom-built c++ hardware driver, integrated with ROS2 control framework, providing a robust interface between high-level trajectory planning and low-level serial communication.

<p align="center">
  <img src="images/Jaw.heic" alt="Jaw PixArm" width="400"/>
</p>
---

## Quick Start Guide

Follow these steps every time you power on the robot and connect the U2D2 to your laptop.

### 1. Hardware Preparation
1. Connect the **12V Power Supply** to the Dynamixel power hub.
2. Connect the **U2D2 USB** to your dev machine (laptop/PC/raspberryPI5).
3. Optional step if you run the project from VM: 
	- In VMware, ensure the USB device is connected to the **Linux Guest** (Virtual Machine > Removable Devices > Connect).

### 2. Permissions & Port Optimization
(Optional if you run the project from a VM): Before launching, you must give the VM permission to talk to the serial port and reduce latency to prevent "Overrun" warnings (this is necessary if you run ubuntu on a VM).

```bash
# Grant USB permissions
sudo chmod 666 /dev/ttyUSB0

# Set low latency mode (Reduces Read/Write time from 16ms to ~1-2ms)
sudo apt update && sudo apt install setserial
sudo setserial /dev/ttyUSB0 low_latency

# Build & source
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch
ros2 launch my_robot_bringup my_robot.launch.xml

# Move the robotic arm method 1
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  points: [{
    positions: [0.2, 0.2, 0.2, 0.0, 0.0, 0.0],
    time_from_start: {sec: 2, nanosec: 0}
  }]
}" -1

# Move the robotic arm method 2 with GUI sliders
ros2 run joint_state_publisher_gui joint_state_publisher_gui

```

