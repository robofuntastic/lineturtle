# Lineturtle ROS2 Package

Welcome to the Lineturtle ROS2 package repository! Lineturtle is an exciting project by RoboFuntastic that lets you build and control your very own drawing robot using ROS2 and MicroROS. This repository contains all the necessary code, resources, and instructions to get you started on this creative and educational journey.

[![Lineturtle Robot](images/lineturtle_render_1.png)
](https://www.youtube.com/@RoboFuntastic)
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
3. Clone teleop_twist_joy repository (becuse lineturtle depends on the package) to your ROS2 workspace:
   ```shell
   git clone https://github.com/ros2/teleop_twist_joy.git -b foxy
4. Clone micro-ROS-Agent repository (becuse lineturtle depends on the package) to your ROS2 workspace:
   ```shell
   git clone https://github.com/micro-ROS/micro-ROS-Agent.git -b foxy
5. Build the ROS2 package and install dependencies:
   ```shell
   cd  ..
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/local_setup.bash
6. Configure your robot's hardware and connections as described in the guide.


## Contributing
We welcome contributions and suggestions from the community! If you find issues, want to suggest enhancements, or contribute to the codebase, please check out [ROS Discourse](https://discourse.ros.org/t/lineturtle-drawing-robot-based-on-esp32-microros-and-ros-2-by-robofuntastic/32940).

## Support
If you have questions, need assistance, or want to share your Lineturtle experiences, feel free to join our Discord Community.

## License
This project is licensed under the MIT License.

Unlock the world of robotics, creativity, and learning with Lineturtle by RoboFuntastic. Start building and exploring today!

## [RoboFuntastic YouTube Channel](https://www.youtube.com/@RoboFuntastic)
## Udemy Course
## [ROS Discourse Community](https://discourse.ros.org/t/lineturtle-drawing-robot-based-on-esp32-microros-and-ros-2-by-robofuntastic/32940)
