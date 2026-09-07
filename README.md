# Task 12.2 – Closed-Loop PID Control

## Overview

This task is an upgrade of our previous Task 7.2 Maze Solver.

In Task 7.2, the robot used open-loop control. In this task, we will upgrade it to a closed-loop control system using continuous "/odom" feedback and PID controllers.

The existing Gazebo simulation, autonomous maze-solving logic, and custom action servers must still work.

The required action servers are:

- "movement_x"
- "movement_yaw"

---

## Main Requirements

The robot must:

- Use "/odom" feedback continuously.
- Dynamically calculate "/cmd_vel".
- Reach target distances accurately without overshoot.
- Reach target angles accurately without oscillation.
- Keep straight during linear movement using heading correction.
- Support runtime PID parameter tuning.
- Include a watchdog timer for safety.

The required PID controllers are:

### 1. Linear Distance PID – Member 2

Modify "movement_x".

You need to:

- Use "/odom" to calculate the distance error.
- Implement Linear Distance PID.
- Dynamically calculate the linear velocity.
- Stop when the target distance is reached.

Also implement:

- Target Deadzone
- Integral Anti-Windup
- Conditional Integration
- Zero-Crossing Reset
- Output Clamping

### 2. Rotational Yaw PID – Member 3

Modify "movement_yaw".

You need to:

- Get the robot yaw from "/odom".
- Calculate the angular error.
- Normalize the angle error.
- Implement Yaw PID.
- Dynamically calculate angular velocity.
- Stop accurately at the target angle.

Also implement:

- Target Deadzone
- Integral Anti-Windup
- Conditional Integration
- Zero-Crossing Reset
- Output Clamping
- Angle Normalization

### 3. Heading Correction PID – Member 4

Implement heading correction during linear movement.

You need to:

- Save the starting heading.
- Continuously read the current heading from "/odom".
- Calculate the heading error.
- Use PID to calculate an angular correction.
- Add the correction to the same "/cmd_vel" Twist used for linear movement.

The goal is to keep the robot moving straight and prevent drifting.

### 4. Dynamic Parameters + Watchdog – Member 5

You need to:

- Add dynamic "Kp", "Ki", and "Kd" parameters.
- Allow PID parameters to be changed at runtime.
- No recompiling should be required for tuning.
- No node restart should be required for tuning.
- Implement a Watchdog Timer.
- Stop the robot safely if control updates or required feedback stop arriving.

### 5. Integration + Testing – Member 1

Member 1 is responsible for preparing and maintaining the project before and during the Task 12.2 upgrade.

Responsibilities include:

- Prepare the Task 12.2 repository.
- Fix and verify the existing Task 7.2 implementation before starting the upgrade.
- Make sure the original Task 7.2 Gazebo simulation and autonomous maze-solving logic work correctly.
- Resolve existing Git/code conflicts and integration issues.
- Create and maintain the "task12.2" branch from the working Task 7.2 version.
- Review team members' work.
- Merge Pull Requests into "task12.2".
- Handle merge conflicts when integrating team members' branches.
- Build the complete project after integration.
- Run the Gazebo simulation.
- Test the complete closed-loop maze solver.
- Fix integration problems and ensure all Task 12.2 requirements work together.
- Keep the final "task12.2" branch stable and ready to be merged into "main".

---

## Git Workflow

* The original working Task 7.2 version is on "main".

* The Task 12.2 work is done on: "task12.2"

### Each team member must create their own branch from task12.2.

Suggested branches:

       member2-linear-pid
       member3-yaw-pid
       member4-heading-correction
       member5-params-watchdog


Member 1 will work directly on task12.2 for integration and final testing.

---

## How to Get the Repository

### 1. If you already have the repository

If you already have "maze1_solver" on your computer:

        cd maze1_solver
        git fetch
        git checkout task12.2
        git pull
        git checkout -b yourbranch_name
        git push -u yourbranch_name

### 2. If this is your first time downloading the repository


        git clone https://github.com/ayavmohamed/maze1_solver.git
        cd maze1_solver
        git checkout task12.2
        git pull
        git checkout -b yourbranch_name
        git push -u origin yourbranch_name

---

## Before Starting Work

Always make sure you are on your assigned branch:

         git branch

The branch with "*" is your current branch.

**Do not work directly on "main"!**

---

## Saving and Uploading Your Work

After making your changes:

        git add .
        git commit -m "Describe your changes"
        git push

Then go to GitHub and create a Pull Request:
Your branch → task12.2

Do not push directly to "main".

---

## Pull Request
When you finish your part:

1. Push your branch to GitHub.
2. Open a Pull Request.
3. The base branch must be "task12.2".
4. Your branch should be the compare branch.
5. Explain briefly what you changed.

For example:

"member2-linear-pid" → "task12.2"

Do not make a Pull Request directly to "main".

---

## Important

- Each member works only on their assigned branch.
- Do not delete or rename the assigned branches.
- Do not push directly to "main".
- Do not create another branch unless the team agrees.
- Keep the existing Task 7.2 functionality working.
- Test your changes before creating the Pull Request.

The final integrated and tested version will be on "task12.2" before it is merged into "main".