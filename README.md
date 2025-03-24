# RobétArmé SFR Integration - WP5.2 - ROS2

## Introduction
This package is designed to run a finite state machine that executes a set of commands for shotcrete application or surface finishing. We have developed several independent ROS 2 libraries, but the task execution is built and launched using ROS Humble. Only the `wp5-ros2-interface` package contains ROS 2 components in its source code.

## Structure
The system is composed of five main packages:

- **wp5_robotic_arms**: Contains all functions necessary for low-level robot control, including inverse kinematics, velocity calculations, Jacobian computations, etc.
- **wp5_ros2_interface**: Manages ROS 2 topic communication.
- **wp5_task**: Acts as the task manager, integrating all the previous packages and executing the finite state machine to perform shotcrete application or surface finishing.

## Finite State Machine

This document describes the **Finite State Machine (FSM)** governing the robotic system's workflow.

### 1. Initializing → Planning → Feasibility Check → Ready
- **Planning Phase:**
  - The system starts by checking the **mesh or point cloud (PCD)** for surface analysis.
  - It performs **analysis and processing** to generate a **UV map**.
  - From the UV map, a **feature space** is created.
  - A **planner** is developed within the **2D feature space polygon**.
  - A **mapping algorithm** is used to link the **original space** with the **feature space**.

- **Feasibility Check:**
  - Before running the robot, we compute **N different inverse kinematics (IK) solutions** for the **first position**.
  - We select an IK solution that allows the robot to **execute the entire path** without issues.
  - An **offline simulation** of the mapping is performed, verifying:
    - The selected **IK solution remains feasible** throughout the task.
    - There are **no collisions** between the robot, the target, or predefined static objects (**walls, mobile platforms, etc.**).
    - The robot remains **far from singularities**.
  - If all conditions are met, an **interpolated joint path** is generated using **PyBullet**, ensuring a **collision-free transition** from the current joint state to the selected first joint position.

### 2. Ready → Executing
- In **Executing**, the system performs **shotcreting or surface finishing**.
- Both tasks operate in a **closed-loop system**, ensuring that the **end effector (EEF) maintains the desired position and speed**.
- The closed-loop system is part of a **coupled dynamical system**, meaning the control of the EEF is influenced by the interaction between the robot’s motion and external forces, ensuring smooth and stable operation.
- **Class mapping** is used to adapt the **EEF’s speed across different spaces**, ensuring proper velocity regulation when transitioning between the **original space and the feature space**.

### 3. Executing ↔ Shared Control
- In **Shared Control**, the system **pauses autonomous behavior** to allow **teleoperation**.
- When teleoperation stops, the system **resumes autonomous execution**.

### 4. Executing → Homing
- If execution fails (**Not OK**), the system enters **Homing** for recovery.
- If successful (**OK**), it may still transition to **Homing** before exiting.

### 5. Homing → Exit
- The system finds a **collision-free path** from the current joint state to the homing position, similar to the initial joint position planning.
- The system stops after completing homing.

![Alt text](FSM.png)
**Legend:**
- 🔵 **Blue arrows** indicate normal state transitions.
- 🔴 **Red arrows** represent error-handling paths.


## Getting Started
To clone the repository with the required submodules, use:

```bash
git clone --recurse-submodules git@github.com:bonato47/robetarme_integration_wp5.2.git
```

## Connecting to a Real UR5 Robot
To connect to the Ridgeback computer and run ROS 2 communication with the UR5 robot and force torque sensor:

⚠ **Warning:** Ensure you are connected to the robot via Ethernet to the top switch with IP address `10.42.0.23`.

```bash
ssh -X administrator@10.42.0.187
cd ~/ur5_lasa/docker_interface_ros2 
docker compose build  # Run this only the first time
docker compose up
```

## Connecting to UR5 in Simulation
For simulation instructions, please refer to the GitHub repository epfl_lasa/ur5_lasa.

## Running the Shotcrete/Surface Finishing Task
```bash
cd ~/robetarme_integration_wp5.2
docker compose build  # Run this only the first time
docker compose up -d 
docker exec -it ros_humble_a10_development_container bash
colcon build
source install/setup.bash
ros2 launch wp5_tasks main_task.py taskType:=surface_finishing  # Use 'shotcrete' or 'surface_finishing'
```

## Saving a New Target
To save a new target, you need a ROS 2 marker topic to place the mesh or PCD in the correct position. You can either:
- Use `fakeMarker.py` (a GUI tool to place the pose manually).
- Use the `optitrack_ros_interface` package to stream pose data from OptiTrack (`git@github.com:epfl-lasa/optitrack_ros_interface.git`).

Then, launch the PCD or mesh loader to save the new target position.

```bash
cd ~/robetarme_integration_wp5.2
docker exec -it ros_humble_a10_development_container bash
python src/wp5_perception/wp5_perception/fakeMarker.py  # Launches a GUI to place the pose
ros2 launch wp5_perception load_pointcloud.launch.py  # Use 'mesh_pointcloud.launch.py' for meshes
# Press the 'Update' button in the GUI; a message in the terminal should confirm that the target has been saved.
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

