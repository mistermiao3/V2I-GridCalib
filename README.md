V2I-GridCalib is a real-time, online calibration framework for Vehicle-to-Infrastructure (V2I) cooperative perception systems. This project provides the official implementation for the paper: "V2I-GridCalib: A Real-time Framework for Online Vehicle-to-Infrastructure Calibration using Scale-Adaptive Rasterized Views".
Overview
The core mission of V2I-GridCalib is to provide a robust and accurate 6-DoF pose estimate of a vehicle within the coordinate frame of a roadside unit (RSU) on-the-fly, without the need for pre-deployed targets. Our framework uniquely addresses the critical challenge of observational scale mismatch in dynamic urban environments.
The calibration pipeline operates in two main stages:
Stage 1: Directional Handshake & Data Pre-selection: The system first efficiently determines the vehicle's approach vector using a lightweight visual handshake. This allows for the dynamic extraction of a relevant, fan-shaped point cloud subset from the RSU's panoramic 360° sensor data, drastically reducing computational overhead.
Stage 2: Multi-Constraint Pose Estimation: To handle scale variations, we introduce a distance-adaptive rendering strategy that generates an information-rich, four-channel rasterized view of the point cloud, with a scale optimized for the vehicle's current distance. The final vehicle pose is then estimated within a particle filter framework, which robustly fuses two complementary geometric constraints: a 2D-3D reprojection constraint (from camera-to-rasterized-view matches) and a 2D-2D epipolar constraint (from camera-to-camera matches).
Ongoing Development & Future Updates
This repository is under active development. Our goal is to continuously improve the robustness, accuracy, and usability of the V2I-GridCalib framework. Future updates will include:
Code refactoring for better readability and easier deployment.
Support for a wider range of LiDAR and camera sensors.
Integration with ROS 2 for real-time applications.
Detailed documentation and tutorials.
We welcome contributions and feedback from the community! Please feel free to open an issue or submit a pull request.

Stay tuned for code refactoring and optimization！
