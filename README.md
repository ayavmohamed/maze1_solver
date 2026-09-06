# Task 7 - Autonomous Maze Solver

## Overview

This project is a ROS 2 maze solver using TurtleBot3 Burger in Gazebo.

The robot can move autonomously through the maze by:

* Moving forward a specific distance.
* Rotating by a specific yaw angle.
* Controlling the red walls/gates using a ROS service.
* Using "/odom" to measure the robot's movement.
* Sending velocity commands through "/cmd_vel".
* Completing the maze automatically using a high-level action client.

The project uses:

* ROS 2 Jazzy
* Gazebo Harmonic
* TurtleBot3 Burger
* Python
* Custom ROS 2 Actions
* ROS 2 Services
* ROS 2 Topics

---

## How the System Works

The project is divided into three main parts.

### 1. Wall Control

"wall_retraction_service.py"

This node controls the red walls in Gazebo.

The service is:

/toggle_walls_1_2

Service type:

std_srvs/srv/SetBool

When the request is "True":

* Wall 1 goes up.
* Wall 2 goes down.
* This opens Gate 1.

When the request is "False":

* Wall 1 goes down.
* Wall 2 goes up.
* This opens Gate 2.

---

### 2. Movement Action Server

"robot_control/action_server.py"

This node provides two custom Action Servers:

/movement_x
/movement_yaw

"movement_x" moves the robot a requested distance.

It:

* Waits for "/odom".
* Saves the starting position.
* Publishes velocity commands on "/cmd_vel".
* Calculates how far the robot has moved using "/odom".
* Publishes feedback during movement.
* Stops when the requested distance is reached.
* Has timeout and stall handling.

"movement_yaw" works in a similar way for rotation.

It:

* Uses "/odom" to track the robot's yaw.
* Publishes angular velocity through "/cmd_vel".
* Provides feedback.
* Stops when the requested rotation is reached.
* Handles timeout and missing odometry.

---

### 3. High-Level Action Client

"maze_robo/action_client.py"

This is the main controller of the maze.

The important method is:

solve_maze()

It automatically performs the required movement and wall-control operations by calling the action servers and wall service in the correct order.

---

## Custom Interfaces

**MoveRobotX Action**

        float64 distance
        ---
        bool success
        string message
        ---
        float64 current_distance

The "distance" is the requested movement distance in meters.

---

**RotateRobotYaw Action**

        float64 yaw
        ---
        bool success
        string message
        ---
        float64 current_yaw

The "yaw" is the requested rotation in radians.

For example:

1.5708

is approximately 90 degrees left.

-1.5708

is approximately 90 degrees right.

---

## Building the Workspace

Open a terminal and run:

       cd ~/yourworkspace
       colcon build --symlink-install
       source install/setup.bash

If the build is successful, the packages are ready to run.

---

## Running the Project

**You need three terminals.**

#### Terminal 1 - Start Gazebo

       cd ~/training_ws
       source install/setup.bash
       ros2 launch maze_control maze_simulation_tb3.launch.py

This starts:

* Gazebo
* TurtleBot3 Burger
* Maze world
* Wall retraction service
* Maze timer

Wait until the simulation is fully loaded.

---

#### Terminal 2 - Start the Action Server

Open another terminal:

        cd ~/training_ws
        source install/setup.bash
        ros2 run robot_control action_server

This node handles:

/movement_x
/movement_yaw

---

### Terminal 3 - Start the Maze Controller

Open a third terminal:

        cd ~/training_ws
        source install/setup.bash
        ros2 run maze_robo action_client

The action client will automatically start solving the maze.

No manual "/cmd_vel" commands are required.


#### Important Notes

* Make sure Gazebo is fully running before starting the action server.
* The action server must be running before starting the action client.
* Always run "source install/setup.bash" after building the workspace.
* The robot uses "/odom" for movement and rotation feedback.
* The final movement distance may need to be adjusted slightly depending on the simulation setup.
