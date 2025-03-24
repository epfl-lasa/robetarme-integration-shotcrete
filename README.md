# RobétArmé SFR Integration shotcrete - WP5.2 - ROS2

## Introduction
This package is designed for the integration with KUL to perform shotcrete with their planner for the Robetarme project.

## Structure
The system is composed of three main packages:

- **wp5_robotic_arms**: Contains all functions necessary for low-level robot control, including inverse kinematics, velocity calculations, Jacobian computations, etc.
- **wp5_ros2_interface**: Manages ROS 2 topic communication.
- **wp5_task**: Acts as the task manager, integrating all the previous packages and executing the finite state machine to perform shotcrete application.


## Getting Started
To clone the repository with the required submodules, use:

```bash
git clone --recurse-submodules git@github.com:epfl-lasa/robetarme_integration-shotcrete.git
```



## Configuration Files
There are six different configuration files:

- **arm_robot_config**: Defines the characteristics of the robotic arm (should not be modified).
- **ros_interface_config**: Defines ROS 2 topic parameters (should not be modified).
- **target_config**: Specifies whether to use a mesh (`True`) or PCD (`False`) for mapping.
- **tools_config**: Defines the offset between the end-effector and the target for different tasks.
- **tracIk_config**: Configures the inverse kinematics solver, including tolerance and mode.


All the control is realatively from the base_link. 
the urdf of the sfr arm need to be upload as:
/home/lasapc/Documents/ROS2-robetarme/robetarme_integration_wp5.2/src/wp5-robotic-arms/urdf/robetarme.urdf
and can be saved from Cobod pkg with:
ros2 run xacro xacro src/manipulator/Cobod_RoBetArme_ROS2_Description/urdf/robetarme.urdf.xacro > src/manipulator/Cobod_RoBetArme_ROS2_Description/urdf/robetarme_shot.urdf name:=robetarme use_mock_hardware:=true ee_type:=shotcret
e
