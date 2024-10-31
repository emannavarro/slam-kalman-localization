# SLAM Kalman Localization

**CMPE 185 Project**

Welcome to the SLAM Kalman Localization project! This repository is dedicated to implementing and testing SLAM (Simultaneous Localization and Mapping) techniques using Kalman filters. We are utilizing a submodule as a sandbox environment to experiment with various simulations.

## Project Description

This project simulates a robot model to implement and test a SLAM algorithm using Webots and ROS2. SLAM, or Simultaneous Localization and Mapping, is a technique that enables a robot to map an unknown environment while simultaneously tracking its position. Our approach uses lidar and camera sensors to process environmental data and create an accurate representation of the surroundings.

To enhance accuracy, we integrate a Kalman filter that merges real-time sensor data, addressing sensor noise and allowing the robot to produce more accurate position estimations. By combining SLAM with the Kalman filter, the robot achieves improved mapping, localization, decision-making, and navigation capabilities, even in dynamic or noisy environments.

### Problem Statement

This project addresses the challenge of simulating real-time SLAM with noisy sensor inputs from both lidar and camera sources. We start by testing Differential drive and Ackerman-like models to assess how different kinematic models affect SLAM performance. Integrating data from multiple sensors poses computational demands, particularly in real-time, and requires a high-performance computer with a powerful processor and graphics card.

### Approach

1. **Robot Models**: Testing with Differential drive and Ackerman models to evaluate the influence of kinematics on SLAM performance.
2. **Sensor Data Processing**: Lidar serves as the primary environmental sensor, with camera data integrated to enhance spatial representation. The Kalman filter will merge data from both sources to provide accurate, noise-resistant position estimations.
3. **Simulation Environments**: Testing will occur in both indoor and outdoor terrains to observe SLAM performance across various surfaces.
4. **Validation**: We will compare the robot’s predicted position and generated map with ground truth data, benchmarking real-time performance to evaluate accuracy and efficiency.



## Installation Guide

We are using [this installation guide](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html) for setting up ROS 2 and Webots on Ubuntu 22.04.5 LTS. The guide provides step-by-step instructions for installing the necessary components for Webots simulation with ROS 2 on a Windows machine with WSL (Windows Subsystem for Linux).

## Getting Started

To get a local copy of the project, you can clone the repository and initialize the submodule by using:


```bash
git clone --recurse-submodules git@github.com:emannavarro/slam-kalman-localization.git
```

