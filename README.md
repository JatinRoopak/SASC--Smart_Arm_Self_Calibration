# SASC: Smart Arm Self-Calibration 

**Current Status:** Alpha (Work in Progress)  
**Supported Platform:** ABB IRB 6640 (Simulation)  
**Middleware:** ROS 2 Humble Hawksbill  

---

## Overview

SASC is a closed-loop perception and control framework designed to autonomously calibrate robotic manipulator positioning using visual feedback. By leveraging an eye-in-hand camera setup and ArUco markers, the system measures real-time kinematic errors (RMSE) and applies dynamic corrections to the robot's motion planning stack, eliminating control drift without manual intervention.

![System Architecture Diagram](docs/images/architecture_placeholder.png)
*(Placeholder: Add a diagram showing data flow between Camera, Commander, and Calibration Node)*

---

## Key Features

* **Closed-Loop Visual Servoing:** Real-time tracking of ArUco markers using OpenCV and ROS 2.
* **Automated Error Calculation:** Calculates Root Mean Square Error (RMSE) between the robot's end-effector frame and the visual target.
* **Dynamic Parameter Tuning:** Automatically updates the safety distance parameters in real-time based on calculated drift.
* **Safety-Critical Design:** Implements non-blocking, multithreaded architecture to ensure collision avoidance during calibration routines.

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
Usage Guide
To run the full self-calibration loop, open three separate terminals. Note: Remember to run source install/setup.bash in every terminal.

Terminal 1: The "Brain" (Calibration Node)
Starts the error calculator. It will wait for data samples.

Bash
```bash
ros2 run sasc error_calibration
```
Terminal 2: The Simulation
Launches Gazebo, MoveIt, and the Robot State Publisher.

Bash
```bash
ros2 launch sasc arm_commander.launch.py
```
Wait for the message: "You can start planning now!"

(Placeholder: Screenshot of the Gazebo Environment with Robot and ArUco Marker)

Terminal 3: The "Body" (Commander Node)
Sends commands to move the robot to the target.

Bash
```bash
ros2 run sasc arm_commander
```
Verification
Move the Marker: Drag the ArUco marker to a new location in Gazebo.

Observe Tracking: The robot will plan a path and stop at the defined safety_dist.

Auto-Correction: * After collecting 10 data samples, Terminal 1 will compute the drift.

Terminal 3 will receive the correction and print: CALIBRATION APPLIED.

Subsequent movements will have reduced systematic error.

Known Issues
Robot Compatibility: Currently hardcoded for the ABB IRB 6640. Support for generic URDFs is planned for future releases.

TCP Offset: Users may observe a static offset (~10mm) between visual depth and kinematic depth. This is a physical offset due to the camera mounting position defined in the URDF, not a software error.
