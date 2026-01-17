SASC: Smart Arm Self-Calibration System
Current Status: Work in Progress (Alpha) Supported Platforms: ABB IRB 6640 (Simulation)

SASC is a closed-loop perception and control framework designed to autonomously calibrate robotic manipulator positioning using visual feedback. By leveraging an eye-in-hand camera setup and ArUco markers, the system measures real-time kinematic errors (RMSE) and applies dynamic corrections to the robot's motion planning stack, eliminating control drift without manual intervention.

![Placeholder: System Architecture Diagram showing Data Flow between Camera, Commander, and Calibration Node]

Key Features
Closed-Loop Visual Servoing: Real-time tracking of ArUco markers using OpenCV and ROS 2.

Automated Error Calculation: Calculates Root Mean Square Error (RMSE) between the robot's end-effector frame and the visual target.

Dynamic Parameter Tuning: Automatically updates the safety distance parameters in real-time based on calculated drift.

Safety-Critical Design: Implements non-blocking, multithreaded architecture to ensure collision avoidance during calibration routines.

Prerequisites
Before cloning the repository, ensure you have the following installed:

OS: Ubuntu 22.04 LTS (Jammy Jellyfish)

Middleware: ROS 2 Humble Hawksbill

Simulation: Gazebo Classic 11

Planning: MoveIt 2

Dependencies: cv_bridge, image_transport, tf2_ros

⚠️ Critical Setup: Custom Gazebo Models
This simulation relies on custom ArUco marker models that are not included in the standard Gazebo library. You must manually install these models for the simulation to load correctly.
Performance Tip: Download Gazebo Models Locally
Gazebo often hangs or fails to load because it tries to download standard models (like sun, ground plane, etc.) from the internet at runtime. To fix this, download the full model database locally:

Bash

# 1. Create the directory if it doesn't exist
mkdir -p ~/.gazebo/models

# 2. Clone the official model repository
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models/

Navigate to the models directory inside this repository.

Copy the aruco_marker folder to your local Gazebo models directory.

Bash

# Example command (adjust paths as necessary)
cp -r ~/sasc_ws/src/sasc/models/aruco_marker ~/.gazebo/models/
Note: If the ~/.gazebo/models directory does not exist, create it using mkdir -p ~/.gazebo/models.

Installation
Create a Workspace (if you haven't already):

Bash

mkdir -p ~/sasc_ws/src
cd ~/sasc_ws/src
Clone the Repository:

Bash

git clone <YOUR_REPO_URL>
Install Dependencies:

Bash

cd ~/sasc_ws
rosdep install --from-paths src --ignore-src -r -y
Build the Package:

Bash

colcon build --packages-select sasc
source install/setup.bash
Quick Start Guide
To run the full self-calibration loop, you will need three separate terminal windows. Ensure you source the workspace in every terminal (source install/setup.bash).

Terminal 1: The "Brain" (Calibration Node)
Start the calibration node first. It will wait for incoming data samples.

Bash

ros2 run sasc error_calibration
Expected Output: [INFO]: Calibration Node Started. Expecting Safety Distance: 0.40 meters

Terminal 2: The Simulation (Gazebo & MoveIt)
Launch the simulation environment and the robot controller.

Bash

ros2 launch sasc arm_commander.launch.py
Wait until you see "You can start planning now!" in the MoveIt logs.

![Placeholder: Screenshot of the Gazebo Environment with Robot and ArUco Marker]

Terminal 3: The "Body" (Commander Node)
This node will actuate the robot to track the marker.

Bash

ros2 run sasc arm_commander
Verification
Move the ArUco marker inside the Gazebo simulation.

The robot will track and approach the marker, stopping at the safety distance.

After 10 samples, Terminal 1 will calculate the error and publish a correction.

Terminal 3 will receive the correction and print: ✅ CALIBRATION APPLIED.

System Architecture
The system consists of three primary nodes communicating over ROS 2 topics:

Camera Reader (camera_reader.cpp):

Subscribes to raw image data.

Detects ArUco markers and computes the TF transform relative to the camera frame.

Publishes the marker pose.

Arm Commander (arm_commander.cpp):

Acts as the central controller.

Receives marker poses and plans trajectories using MoveIt 2.

Maintains the safety_dist parameter and updates it dynamically based on feedback.

Error Calibration (error_calibration.cpp):

Collects a batch of position data (Robot Frame vs. Camera Frame).

Computes the Root Mean Square Error (RMSE).

Publishes a float correction value to /sasc/correction to close the control loop.

![Placeholder: Data Flow Graph (Nodes and Topics)]

Limitations & Known Issues
Robot Compatibility: Currently hardcoded for the ABB IRB 6640. Support for generic URDFs via parameterization is planned.

Tool Center Point (TCP) Offset: Users may observe a static offset (~10mm) between visual depth and kinematic depth. This is a physical offset due to the camera mounting position defined in the URDF, not a software error.

<div id="top"></div>

<!-- 

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: October, 2022.                                                                 #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

-->

<!--

  README.md TEMPLATE obtined from:
      https://github.com/othneildrew/Best-README-Template
      AUTHOR: OTHNEIL DREW 

-->

<!-- HEADER -->
<br />
<div align="center">
  <a>
    <img src="media/header.jpg" alt="header" width="651" height="190.5">
  </a>

  <br />

  <h2 align="center">ROS2.0 ROBOT SIMULATION - ROS2.0 Humble</h2>

  <p align="center">
    IFRA (Intelligent Flexible Robotics and Assembly) Group
    <br />
    Centre for Robotics and Assembly
    <br />
    Cranfield University
  </p><div id="top"></div>

<!-- 

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: October, 2022.                                                                 #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

-->

<!--

  README.md TEMPLATE obtined from:
      https://github.com/othneildrew/Best-README-Template
      AUTHOR: OTHNEIL DREW 

-->

<!-- HEADER -->
<br />
<div align="center">
  <a>
    <img src="media/header.jpg" alt="header" width="651" height="190.5">
  </a>

  <br />

  <h2 align="center">ROS2.0 ROBOT SIMULATION - ROS2.0 Humble</h2>

  <p align="center">
    IFRA (Intelligent Flexible Robotics and Assembly) Group
    <br />
    Centre for Robotics and Assembly
    <br />
    Cranfield University
  </p>
</div>


