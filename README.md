# UR10e Robot Arm with Sanding Behavior Tree in ROS 2

This repository contains the implementation of a simulated environment for a UR10e robot arm using ROS 2. The robot arm performs sanding tasks through a behavior tree, including home position, quality check of the surface, sanding movement, and returning to the home position. The implementation uses MoveIt2 for path planning, Rviz for visualization, and action clients for task execution.

## Features
- **Simulation of UR10e robot arm in Gazebo**
- **MoveIt2 integration** for motion planning and visualization in Rviz
- **Behavior tree** to handle sequential task execution (e.g., home position, quality check, sanding, back to home)
- **Force feedback** for end-effector interaction (via `/jt` topic)

## Requirements
- **ROS 2 (Humble)**
- **Gazebo**
- **MoveIt2**
- **Behavior Trees** (Action clients, servers)
- **ros2_control** and related plugins

## Setup
![image](https://github.com/user-attachments/assets/877d53f1-d482-49c4-9e8b-384847ca115f)


### 1. Launch the UR10e Robot Arm in Gazebo
Start the simulated environment with the UR10e robot arm in Gazebo:

```bash
ros2 launch ur_description_pkg gazebo.launch.py
```

### 2. Launch MoveIt2 for Path Planning and Rviz Visualization
Start MoveIt2 for robot motion planning and visualization in Rviz:

```bash
ros2 launch robot_moveit moveit.launch.py
```

### 3. Run the Robot Server for Task Execution
Run the robot server node for executing tasks and robot path planning using Cartesian paths:

```bash
ros2 run robot_moveit robot_server_node
```

### 4. Run the Behavior Tree Execution for Sanding Task
Run the behavior tree sequence for robot task execution:

```bash
ros2 run control_robot sanding
```

This will execute the following sequence:
- Move to home position
- Perform surface quality check
- Execute sanding movement
- Return to home position

### 5. Monitor Force Feedback
To monitor the force feedback from the end effector, run:

```bash
ros2 topic echo /jt
```

## Challenges Faced
1. **Finding a force sensor plugin for ROS 2 in Gazebo:**
   - Took significant time to search for a compatible force sensor plugin for ROS 2 simulation.

2. **Understanding Behavior Trees and ROS 2 Integration:**
   - Faced multiple errors while integrating behavior trees with action clients.
   - The official ROS 2 documentation provided crucial help in successfully implementing the sequence node.

## Video Demonstration
Watch the video demonstrating the implementation of the sanding behavior tree and robot task execution:
[Watch Video](https://www.youtube.com/watch?v=TZqeba6U9-w)

## License
This repository is licensed under the MIT License. See the LICENSE file for more details.

