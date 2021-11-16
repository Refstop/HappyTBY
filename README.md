# HappyTBY

Mando AI Autonomous Racer Contest (~2021.11.27)

Autonomus Driving Car Stack benchmarked [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) stack.

Sponsored by Mando, aMaps, Halla Univ.



**Use `catkin build` instead of `catkin_make`**

## Implements In Project

- Computer Vision Developments
  - **Line Detection & Following** using OpenCV
  - **Deep Learning(Object Detection)** using yolov4
  - **Obstacle Avoidance** using OpenCV
- SLAM & Navigation Developments
  - **SLAM** using gmapping
  - **Navigation** using move_base and teb_local_planner
- System Developments
  - **System Integration** using ROS
  - **Auto Parking Assistant**

## Prerequisites

- Ubuntu 18.04 (JetPack 4.2+)
- ROS Melodic
- Arduino IDE
- OpenCV 3.2.0+
- ros-melodic-gmapping package
- Move_base & DWA Planner package

## Hardware Specification

- NVIDIA Jetson Nano Kit 4G
- WiFi Module for Jetson Nano
- Arduino MEGA 2560
- aMaps I2C Motor Driver Shield
- sc-mini LiDAR
- CSI-camera Module
- RC Car base

## Team Members

- [TaeHwan, Kim](google.com) : Team leader, Object Detection and Line Detection & Following
- [ByungHee, Choi](https://github.com/Refstop) : ROS Package Build, Object Detection and SLAM & Navigation
- YunA, Park : Obstacle Avoidance, Auto Parking Assist