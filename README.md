# Lineturtle ROS2 Package

Welcome to the Lineturtle ROS2 package repository! Lineturtle is an exciting project by RoboFuntastic that lets you build and control your very own drawing robot using ROS2 and MicroROS. This repository contains all the necessary code, resources, and instructions to get you started on this creative and educational journey.

![Lineturtle Robot](images/lineturtle_render_1.png)

## Features

- `/servo`: Control the servo for marker placement on the drawing surface.
- `/LEDs`: Manage the LEDs mounted on the robot.
- `/left_motor_ticks` and `/right_motor_ticks`: Get motor encoder values for left and right wheels.
- `/battery`: Monitor the battery level of the robot.
- `/cmd_vel`: Control the movement of the robot.

## Getting Started

Follow these steps to get started with your Lineturtle robot:
1. Go to your workspace:
   ```shell
   cd ros2_ws/src/
   
2. Clone this repository to your ROS2 workspace:
   ```shell
   git clone https://github.com/robofuntastic/lineturtle.git
3. Install dependencies
4. Build the ROS2 package and install dependencies:
   ```shell
   cd  ..
   colcon build --symlink-install
5. Configure your robot's hardware and connections as described in the guide.


Contributing
We welcome contributions and suggestions from the community! If you find issues, want to suggest enhancements, or contribute to the codebase, please check out our Contribution Guidelines.

Support
If you have questions, need assistance, or want to share your Lineturtle experiences, feel free to join our Discord Community.

License
This project is licensed under the MIT License.

Unlock the world of robotics, creativity, and learning with Lineturtle by RoboFuntastic. Start building and exploring today!

RoboFuntastic YouTube Channel
Udemy Course
