# Jaw PixArm

6 DoF robotic arm project using **ROS 2 Jazzy Jalisco** and **Dynamixel hardware**.  
This repository contains the source code, control configuration, and launch files needed to build and run the system.
![Jaw PixArm](images/IMG_9680.HEIC)
---

## 📂 Repository Structure
- `src/` – main source code (nodes, launch files, packages)
- `ros2_control/` – ROS 2 control configuration for hardware/simulation
- `dynamixel_hardware/` – drivers and hardware interfaces for Dynamixel servos


---

## 🚀 Prerequisites

- **OS:** Ubuntu 24.04 (Noble Numbat)
- **Middleware:** [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
- **Build tools:** `colcon`

Install common dependencies to control with the ps4 controller:

```bash
sudo apt update
sudo apt upgrade

sudo apt install -y \
  python3-colcon-common-extensions \
  ros-jazzy-ros2-control \
  ros-jazzy-dynamixel-sdk
  ros-jazzy-joy


