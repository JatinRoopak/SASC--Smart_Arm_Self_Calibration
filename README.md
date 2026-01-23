# SASC: Smart Arm Self-Calibration 

**Current Status:** Alpha (Work in Progress)  
**Supported Platform:** ABB IRB 6640 (Simulation)  
**Middleware:** ROS 2 Humble Hawksbill  

---

## Overview

SASC is a closed-loop perception and control framework designed to autonomously align industrial robotic manipulators with visual targets. Unlike open-loop systems that rely on static calibration, SASC utilizes real-time visual servoing to correct kinematic errors in 3D space (Depth, Horizontal, and Vertical).

By bridging the gap between Perception (OpenCV/ArUco) and Control (MoveIt 2), the system dynamically compensates for simulated gear backlash, sensor noise, and TCP drift without manual intervention.

<img width="626" height="530" alt="Screenshot from 2026-01-17 18-23-23" src="https://github.com/user-attachments/assets/e1ca0a85-ab23-4e06-9e36-6d938689fb6d" />

---

## Key Features

* **3D Kinematic Compensation:** Simultaneously corrects errors in X (Depth), Y (Horizontal), and Z (Vertical) axes to sub-millimeter precision.
* **Closed-Loop Visual Servoing:** Implements an adaptive control loop that continuously re-evaluates the target position until the error threshold (<1mm) is met.
* **Dynamic Configuration:** Allows users to define target grasp offsets (offset_x, offset_y, offset_z) at runtime via CLI arguments without recompiling code.
* **Safety-Critical Architecture:** Features velocity scaling (50% limits) and non-blocking execution to prevent collisions during the approach phase.

---

## System Architecture

The framework consists of three synchronized ROS 2 nodes:

1.  **`camera_reader` (Perception):**
    * Detects ArUco markers via an eye-in-hand camera.
    * Broadcasts the marker's 3D pose relative to the camera frame.
    * Logs pose data as: `[Depth(Z) | Right(X) | Down(Y)]`.

2.  **`error_calibration` (The Brain):**
    * Subscribes to robot and camera data.
    * Calculates the 3D error vector (RMSE) over a sampling period (10 frames).
    * Publishes a correction vector to the commander.

3.  **`arm_commander` (Control):**
    * Interfaces with the MoveIt 2 planning pipeline.
    * Executes motion plans based on the target position plus the dynamic error correction.
    * Handles the safety logic and approach velocity.
      
---


## Prerequisites

Before cloning the repository, ensure your environment meets the following requirements:

* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS Distribution:** ROS 2 Humble Hawksbill
* **Simulation:** Gazebo Classic 11
* **Motion Planning:** MoveIt 2
* **Dependencies:** `cv_bridge`, `image_transport`, `tf2_ros`

---

## ⚠️ Critical Setup: Gazebo Models

**Do not skip this step.** This simulation relies on custom ArUco marker models and standard Gazebo assets. Failure to configure this will result in the simulation hanging or the markers failing to spawn.

### 1. Fix Standard Gazebo Models (Prevent Hangs)
Gazebo often hangs while attempting to download models (Sun, Ground Plane) from the internet at runtime. Fix this by downloading the database locally:

```bash
mkdir -p ~/.gazebo/models
git clone [https://github.com/osrf/gazebo_models.git](https://github.com/osrf/gazebo_models.git) ~/.gazebo/models/
```
Note: If ~/.gazebo/models already exists and is not empty, backup or merge the folder.

2. Install Custom ArUco Models
This package contains a custom aruco_marker model required for the calibration target.

Bash

# Copy the custom model from this repo to your Gazebo directory
cp -r ~/sasc_ws/src/sasc/models/aruco_marker ~/.gazebo/models/
Project Structure
Plaintext
```text
sasc/
├── launch_files/
│   └── arm_commander.launch.py   # Main launch file for Robot + MoveIt + Gazebo
├── models/
│   └── aruco_marker/             # Custom SDF model for the visual target
├── src/
│   ├── arm_commander.cpp         # Controller node (Plan & Execute)
│   ├── camera_reader.cpp         # Perception node (OpenCV & TF)
│   └── error_calibration.cpp     # Logic node (RMSE Calculation)
├── CMakeLists.txt
└── package.xml
```
Installation
Create a Workspace

Bash
Clone the Repository
```bash
mkdir -p ~/sasc_ws/src
cd ~/sasc_ws/src
```
Bash
Install Dependencies
```bash
git clone <YOUR_REPO_URL>
```
Bash
Build
```bash
cd ~/sasc_ws
rosdep install --from-paths src --ignore-src -r -y
```
Bash
```bash
colcon build --packages-select sasc
source install/setup.bash
```
## Usage Guide

You can launch the entire system (Robot, Gazebo, MoveIt, and SASC Logic) using a single launch file.

### 1. Standard Launch
Run the system with the default safety distance (0.40m Depth, Centered X/Y):

```bash
ros2 launch sasc sasc_launch.launch.py
```
2. Custom Target Configuration
You can control where the robot stops relative to the box using dynamic arguments. This allows you to test different grasping positions without changing code.

 **Arguments:**
    * ```offset_x```: Target Depth (Forward distance from gripper to box).
    * ```offset_y```: Horizontal Offset (Left/Right).
    * ``` offset_z```: Vertical Offset (Up/Down).

**Example:** Stop 0.5m away, shifted 10cm up and 5cm right:

```Bash
ros2 launch sasc sasc_launch.launch.py offset_x:=0.50 offset_z:=0.10 offset_y:=0.05
```
---
Verification
Move the Marker: Drag the ArUco marker to a new location in Gazebo.

Observe Tracking: The robot will plan a path and stop at the defined safety_dist.

Auto-Correction: * After collecting 10 data samples, Terminal 1 will compute the drift.

Terminal 3 will receive the correction and print: CALIBRATION APPLIED.

Subsequent movements will have reduced systematic error.

Known Issues
Robot Compatibility: Currently hardcoded for the ABB IRB 6640. Support for generic URDFs is planned for future releases.

TCP Offset: Users may observe a static offset (~10mm) between visual depth and kinematic depth. This is a physical offset due to the camera mounting position defined in the URDF, not a software error.
