# ENPM 662 Project 2
  -Varun Lakshmanan - 120169595
  -Nitish Ravisankar Raveendran - 120385506

## Overview
Modeling a robot in solidworks and simulating it in ROS2 Gazebo.
This zip file has multiple packages. 
- project2 
    This is the main package which containsts the model of the robot
- control
    This package contains the open loop controller for the robot
- teleop
    This package contains the teleop node for the robot
- odometry
    This package contains the odometry node for the robot

## Installation and Setup
1. **Build Packages**:
   - Paste the necessary packages into the `src` folder of your workspace.
   - Build the packages using:
     ```
     colcon build
     ```

2. **Source Setup File**:
   - Initialize the environment by sourcing the setup file:
     ```
     source install/setup.bash
     ```

## Launching the Project
1. **Spawn Robot in Gazebo**:
   - Launch the robot simulation in Gazebo using:
     ```
     ros2 launch project2 gazebo.launch.py
     ```

2. **Launch Control Node**:
   - Start the open-loop controller node with:
     ```
     ros2 run control line
     ```

3. **Launch Teleop Node**:
   - Run the teleoperation node using:
     ```
     ros2 run teleop top
     ```

## User Interface and Controls
### Movement Controls:
- `w`: Move forward
- `a`: Turn left
- `s`: Move backward
- `d`: Turn right
- `q`: Force stop movement

### Arm Control:
- `i`: Increase current joint's angle
- `k`: Decrease current joint's angle
- `u`: Select the previous joint
- `o`: Select the next joint

### Exiting the Control:
- Press the `Esc` key to quit the control loop.

## Visualizing the Robot
- Launch RViz for real-time visualization using either:
  ```
  ros2 launch project2 display.launch.py
  ```
  or
  ```
  ros2 launch project2 debug.launch.py
  ```


