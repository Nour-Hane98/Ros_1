# TurtleBot3 ROS1 Activities – UCLM Robot Software Architecture

This repository contains the implementation of a set of activities (Exercises 0 to 8) using the TurtleBot3 robot and ROS1, developed as part of the "Robot Software Architecture" course at the University of Castilla-La Mancha (UCLM). These activities progressively explore topics such as open-loop and closed-loop control, ROS services and actions, geometric motion, and obstacle avoidance using Lidar.


## Activity Overview

### Exercise 1 – Open-loop Movement

- Create a ROS node that moves the robot 1 meter in a straight line.
- **Open-loop control**: Command a linear velocity of 0.05 m/s for 20 seconds.
- No feedback or position checking is used.

---

### Exercise 2 – Closed-loop Control

- Repeat Exercise 1 using **closed-loop control**.
- Use a **proportional controller** (P controller): control signal = Kp × distance error.
- Dynamically adjust velocity based on current position relative to the target.

---

### Exercise 3 – Service-based Movement

- Implement the same movement as in Exercise 2, but using **ROS services**.
- A server node performs the movement; a client node sends service requests while the server is active.

---

### Exercise 4 – Command-based Motion

- Create a node that allows the user to choose between:
  - **Linear movement** or **rotation**
  - The **distance** or **angle**
- Selection is done via a menu or command-line arguments.

---

### Exercise 5 – Motion Using Actions

- Repeat Exercise 4 using **ROS actions** instead of services.
- Provide **feedback messages** to inform the user about progress and completion status.

---

### Exercise 6 – Drawing a Triangle

- Implement a node that makes the robot draw an **equilateral triangle**.
- The **side length L** is provided by the user.
- Combine previous exercises to achieve this.

---

### Exercise 7 – Drawing a Polygon

- Generalize the triangle to draw any **N-sided polygon** with a given side length L.
- When N is large, the robot should approximate a **circle**.
- Test the robustness by checking if the end position matches the starting point after several runs.

---

### Exercise 8 (Optional) – Lidar-based Obstacle Avoidance

- Use the robot’s Lidar to ensure safe linear movement.
- If an obstacle is detected in the path, the node should:
  - Wait until it is safe to move,
  - Notify the user and cancel the movement, or
  - (Advanced) Try to navigate around the obstacle.
 

## Prerequisites

- ROS1 
- TurtleBot3 packages installed
- A working TurtleBot3 robot or simulation environment
- `turtlebot3_simulations` (for Gazebo simulation)




