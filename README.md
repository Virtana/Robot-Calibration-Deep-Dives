# Robot Calibration Deep Dives

**2D Robot Calibration**

This part of the project models a 2D robot with two rotary joints. 
The my_2d_robot package currently contains the urdf describing the 2D robot, the joint state publisher node, my_2d_robo_state_publisher, randomly generates and publishes the state of the robot's rotary joints to the robot_state_publisher and to the joint states subscriber node, as well as joint angle offsets for each joint. 
The joint states subscriber node calculates the position of the end effector and publishes the joints' states (including the offsets) and true end effector position to a yaml file. 
The calibrator node, calibration_2d.cpp, reads the data from the yaml file and calculates the offsets applied using Ceres Solver.

**3D Robot Calibration**

This part of the project models a Fanuc r1000ia80f robot with 6 revolute joints. The relevant Fanuc package is included. 
The publisher node, joint_states_publisher_3d.cpp randomly generates angles for the six revolute joints and publishes them to robot_state_publisher as well as the subscriber node.
The subscriber node, joint_states_subscriber_3d.cpp, uses kdl to calculate the end effector position of the robot and broadcasts the end effector pose.


## Prerequisites
- This package was built using ROS Melodic.
- Ceres Solver
- Eigen3 library
- yaml-cpp library
- boost library

## Installing
Ensure to run the following commands before running `catkin_make` to install all the relevant directories for the 3d robot package:

```
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic
```

## Built With

## Features

## Usage

## Author

Rebecca Gibbon

## License

## Aknowledgements
