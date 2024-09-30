# Pursuit-Evasion Game

Repo game berbasis turtlesim ROS2 Humble untuk Pursuit-Evasion Case

## Prerequisite
- ROS2 Humble
- ROS2 Workspace

<br>

Create pkg inside $yourros2ws/src$

```sh
$ros2 pkg create pursuit_evasion --build-type ament_python --license Apache-2.0 --dependencies geometry_msgs python3-numpy rclpy tf2_ros_py turtlesim$
```
