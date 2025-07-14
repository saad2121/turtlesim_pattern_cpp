# ROS 2 Project: turtlesim_pattern_cpp

This ROS 2 package uses turtlesim to:
- Draw a square with turtle1
- Make turtle2 follow turtle1

## How to build
```bash
cd ~/ros2_ws
colcon build --packages-select turtlesim_pattern_cpp
source install/setup.bash

## How to Launch
ros2 launch turtlesim_pattern_cpp draw_follow_launch.xml
