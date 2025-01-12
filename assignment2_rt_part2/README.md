# Research Track I - Second Assignment Part II
This repository contains the assignment work for the **Research Track I** course, completed by:  
**Rubin Khadka Chhetri**  
**ID: 6558048**

## Table of Contents
- [Introduction](#introduction)
- [Node Details](#node-details)
    - [Robot Controller Node](#robot-controller-node-robot_controller)
- [Repository Structure](#repository-structure)
- [Getting Started (Read Before Action)](#getting-started-read-before-action)
    - [Prerequisites](#prerequisites)
    - [Setup](#setup)
- [Launching Nodes](#launching-nodes)
    - [Launch and spawn robot](#1-run-the-robot_urdf-package)
    - [Launch robot controller node](#2-run-the-robot-controller-node)
- [Implementation Details](#implementation-details)
    - [Robot Controller Node](#robot-controller-node)
- [Summary](#summary)

## Introduction

This repository contains a ROS2 package designed to control a robot in a simulation environment. It implements a single main node:
**Robot Controller Node**. 

The robot is pre-spawned at position `(2, 2)` by another package. The goal of this package is to move the robot by publishing velocity commands to the `/cmd_vel` topic. 

This package demonstrates the use of ROS2 to enable basic robot motion control and interaction with the simulation environment.

**Note**:
- This assignment is completed using **Python**. 
- I removed the c++ part of this assignment as I was not able to run both c++ and python in same package.

## Node Details

### Robot Controller Node (robot_controller)
The **Robot Controller Node** is responsible for controlling the robot's movements in the simulation based on user input. It handles both linear and angular velocity commands and interacts with the robot via the `/cmd_vel` topic.

**Key Functions**:
 - **User Input Handling**: The node prompts the user to input values for the robot's linear and angular velocities.
 - **Movement Execution**: After the velocities are set, the node sends the corresponding movement commands to the robot, causing it to move for one second.
 - **Stop and Reset**: After one second of movement, the robot stops, and the interface is reset, allowing the user to input new movement commands.

## Repository Structure

The root of this repository is the package folder, which contains all necessary files and scripts for running the assignment nodes. When cloning the repository for the first time, place it directly in the `src` folder of your ROS2 workspace.

### Folder and File Overview
- **`/assignment2_rt_part2`**: Contains the main package for controlling the robot in the simulation.
    - `__init__.py`: Initializes the package for Python.
    - `robot_controller.py`: Contains the logic for handling user input and controlling the robot’s movement.

- **`/resource`**: Holds any additional resources or configuration files needed for the package.
    - `assignment2_rt_part2`:  Configuration or resource-related files for the package.

- **`/test`**: Includes test scripts for verifying the package’s functionality and code quality.
    - `test_copyright.py`: Ensures compliance with copyright requirements.
    - `test_flake8.py`: Validates Python code style using flake8.
    - `test_pep257.py`: Ensures adherence to PEP 257 docstring conventions.

- **`/setup.py`**: The script to build and install the package.

- **`/setup.cfg`**: Configuration file for packaging and distributing the package.

- **`/package.xml`**: Lists dependencies and package metadata.

- **`/README.md`**: This file (Documentation).

## Getting Started (Read Before Action)

### Prerequisites
Before proceeding, ensure that **`ROS2`** is installed on your system.<br>
This package has been developed with `ROS2 Foxy`, but it should work with other ROS2 versions as well. 

If you haven’t set up ROS2 yet, you can follow the official guide to install it:<br>
[ROS2 Installation Guide](https://docs.ros.org/)

Additionally, you’ll need `Python 3` package to run this project. Ensure it is installed on your system. If not, you can install it by running:
```bash
sudo apt-get update
sudo apt-get install python3
```
Next, install the required ROS2 packages. These packages are necessary for the robot simulation and its control:
```bash
sudo apt-get install ros-<your-distro>-xacro ros-<your-distro>-joint-state-publisher ros-<your-distro>-gazebo*
```
Replace `<your-distro>` with your ROS2 distribution, for example, `foxy`, `humble`, or `galactic`.

Once **ROS2**, **Python 3**, and the necessary ROS2 packages are installed, you can proceed to set up the workspace.

### Setup
#### 1. Set up your ROS2 workspace

Create a new workspace (or use an existing one) and navigate to its `src` directory:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

#### 2. Install the `robot_urdf` package
Before cloning this repository, you need to get the `robot_urdf` package, which handles starting the simulation and spawning the robot at position `(2, 2)` in the environment. Clone the `robot_urdf` repository into your workspace’s `src` folder:
```bash
git clone https://github.com/CarmineD8/robot_urdf.git
```

#### 3. Switch to the `ros2` branch
Once the `robot_urdf` package is cloned, navigate to its directory:
```bash
cd robot_urdf
```
Change the Git branch from `main` to the `ros2` branch:
```bash
git checkout ros2
```

#### 4. Clone this repository
Now that the `robot_urdf` package is set up, return to the `src` folder and clone this repository into your workspace’s `src` folder:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Rubin-unige/assignment2_rt_part2.git
```

#### 5. Build the workspace
Once both repositories are cloned, navigate back to the root of your workspace and build the packages using the following command:
```bash
cd ~/ros2_ws
colcon build
```

#### 6. Add the Workspace to Your ROS Environment

To ensure that your workspace is sourced automatically every time you start a new terminal session, add it to your `.bashrc` file:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 7. Source the workspace
After building and sourcing, you need to source the workspace manually for the first time in the current terminal session:
```bash
source ~/ros2_ws/install/setup.bash
```

## Launching Nodes

#### 1. Run the `robot_urdf` Package

Before launching your node, ensure the simulation environment is running. Start the `robot_urdf` package to load the simulation and spawn the robot at position `(2, 2)` in Gazebo:
```bash
ros2 launch robot_urdf gazebo.launch.py
```
This will launch the simulation environment and spawn the robot in the Gazebo simulator. Please wait for everything to load properly before proceeding.

#### 2. Run the Robot Controller Node

At this point, you can proceed to run the **Python** version of the `robot_controller` node using following command:
```bash
ros2 run assignment2_rt_part2 robot_controller
```
This will start the **python robot controller** node.

#### 3. Stopping the nodes

To stop the nodes, simply press `Ctrl+C` in the terminal where each node is running. This will terminate the nodes and stop the simulation.

## Implementation Details

### Robot Controller Node
The **Robot Controller Node** is responsible for controlling the robot's movements in the simulation based on user input. It has following key features:
#### 1. Setting Velocities
The `robot_controller` node prompts the user to input two key values for the robot's movement: the **linear velocity** and the **angular velocity**. These values control the robot’s forward/backward speed and its turning rate, respectively.

To ensure that the robot handles the velocities properly, both velocities are constrained to be between `-5` and `5`. This ensures safe and reasonable movement for the robot in the simulation environment.

- **Linear Velocity (x)**:<br> 
The user is prompted to enter the linear velocity, which controls the robot's forward/backward speed. The input is validated to ensure that it is a floating-point number within the range of `-5` to `5`. If the input is invalid, the program asks the user to re-enter a valid value.
```Python
while True:
try:
    linear_x = float(input("Enter the linear velocity (between -5 and 5): "))
    if -5 <= linear_x <= 5:
        break
    else:
        print("Invalid input. Linear velocity must be between -5 and 5.")
except ValueError:
    print("Invalid input. Linear velocity must be a number.")
```

- **Angular Velocity (z)**:<br>
The same process is repeated for the angular velocity input, which controls the robot’s turning speed. It is also validated to ensure it is within the valid range.
```Python
while True:
    try:
        angular_z = float(input("Enter the angular velocity (between -5 and 5): "))
        if -5 <= angular_z <= 5:
            break
        else:
            print("Invalid input. Angular velocity must be between -5 and 5.")
    except ValueError:
        print("Invalid input. Angular velocity must be a number.")
```

#### 2. Publishing User Input
Once the velocities are set, the next step is to publish the values to the `/cmd_vel` topic. This is the primary topic through which velocity commands are sent to the robot in ROS2. The node creates a `Twist` message, which is the standard ROS message used for sending velocity commands. The `linear` and `angular` components of the `Twist` message are populated with the values provided by the user.

To achieve this, a publisher is created that will send the velocity data to the `/cmd_vel` topic. This is done using the `create_publisher` method, which initializes the publisher with the message type `Twist` and the topic name `/cmd_vel`.

Here's how the publisher is set up:
```Python
# Publisher for controlling the robot
self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
```
Next, the Twist message is constructed and populated with the linear and angular velocities entered by the user. The linear velocity is assigned to the `linear.x` field, and the angular velocity is assigned to the `angular.z` field of the `Twist` message.

Here’s how the message is constructed and published:
```Python
# Create a Twist message and populate with user input
robot_vel = Twist() 
robot_vel.linear.x = linear_x
robot_vel.angular.z = angular_z
# Publish to the cmd_vel topic to move the robot
self.pub_cmd_vel.publish(robot_vel)
self.get_logger().info("Moving robot ...")
```
This sends the velocity commands to the robot, allowing it to move according to the user’s input.

#### 3. Stopping the Turtle
After the robot has been commanded to move, we stop it after 1 second by setting both the linear and angular velocities to zero. This halts the robot's movement. To ensure that the robot stops, a stop command is published to the `/cmd_vel` topic.

The following code snippet shows how the robot is stopped:
```Python
time.sleep(1) # Sleep for 1 second
robot_vel.linear.x = 0.0
robot_vel.angular.z = 0.0
self.pub_cmd_vel.publish(robot_vel)
self.get_logger().info("Robot stopped.")
```

## Summary
The `robot_controller` node is a simple ROS2 program that allows users to control a robot's movement by setting linear and angular velocities. The node prompts the user for input, validates the values, and publishes them to the `/cmd_vel` topic, enabling the robot to move. After 1 second, the robot is stopped by setting both velocities to zero.

This program serves as a foundational example of working with ROS2 and can be expanded for more complex projects in the future. By learning how to handle user input, publish messages, and control robot movement, this node provides a solid starting point for developing more advanced ROS2 applications.
