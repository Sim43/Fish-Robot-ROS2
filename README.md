# Biometric-Fish-Robot

ROS2 navigation stack for a biomimetic fish robot with SLAM, localization, and navigation capabilities using Gazebo Harmonic and ROS Jazzy.

## Features

- **SLAM & Localization**: SLAM Toolbox integration for mapping and localization
- **Navigation**: Nav2-based navigation with waypoint following
- **Simulation**: Gazebo Harmonic simulation with custom fish robot model
- **Object Detection**: Python-based object detection node

## Packages

- `bme_ros2_navigation`: Main navigation package with launch files, configs, URDF, and worlds
- `bme_ros2_navigation_py`: Python package for waypoint following and SLAM utilities

## Requirements

- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2, SLAM Toolbox