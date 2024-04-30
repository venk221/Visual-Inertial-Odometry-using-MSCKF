# Visual Inertial Odometry
This repository contains a C++ implementation of a Visual Inertial Odometry (VIO) system using a Multi-State Constraint Kalman Filter (MSCKF). The system fuses data from a stereo camera and an inertial measurement unit (IMU) to estimate the 6DOF pose of a robot or vehicle over time.

## Overview
Visual Inertial Odometry is a sensor fusion technique that combines visual measurements from cameras with inertial data from an IMU to track the 3D motion of a platform. Compared to traditional visual odometry methods that rely solely on camera data, the inclusion of inertial measurements helps improve robustness and accuracy, especially during periods of rapid motion or when visual tracking is challenging.
This implementation uses the MSCKF algorithm, which is a sliding window filter that maintains a constrained set of camera pose states and marginalizes out older poses to control computational complexity. The MSCKF elegantly handles the tight coupling of visual and inertial data, making it an efficient and effective choice for real-time VIO applications.

## Features
- Sensor Fusion: Tightly couples visual and inertial measurements within the MSCKF framework for accurate and robust state estimation.
- Stereo Visual Odometry: Utilizes a calibrated stereo camera to triangulate 3D feature positions and track them over time.
- IMU Preprocessing: Handles IMU bias estimation and applies calibration to the raw inertial data.
- Online Operation: Designed for real-time operation, processing batches of IMU data between camera frames.
- Modular Design: Easily extensible codebase with separate components for IMU processing, vision tracking, and state estimation.
- Evaluation on Datasets: Includes scripts for running the VIO pipeline on the EuRoC dataset and comparing against ground truth poses.

## Installation
### Dependencies:
- ROS (Tested on Melodic)
- Eigen3
- OpenCV (For visual feature tracking)
- Ceres Solver (For visual-inertial optimization)
- Pangolin (For visualization)


## Build Instructions

### Clone the repository:
```
git clone https://github.com/username/visual-inertial-odometry.git
cd visual-inertial-odometry
```

### Build the code using the standard ROS build process:
```
catkin_make
```

## Usage
The VIO system can be run on pre-recorded datasets or with live sensor streams. The following examples demonstrate running on the EuRoC dataset.

### EuRoC Dataset
- Download the EuRoC MAV dataset and update the path to the dataset in the launch file.
- To run the VIO pipeline:
```
source devel/setup.bash
roslaunch vio_msckf euroc.launch
```
- This will load the sensor data from the specified EuRoC sequence and run the VIO algorithm. The estimated trajectory will be published on the /odometry topic and can be visualized using rviz.
Pangolin Simulation
- To run the Pangolin-based simulation, follow these steps:
```
cd Code
python3 vio.py --view --path ./MH_01_easy
```

- This will launch a window simulating the VIO system's operation.

## Trajectory Evaluation
- For ground truth and relative translation error (RTE) calculations, we have used the rpg_trajectory_evaluation repository. To generate the groundtruth.txt file, run the asl_groundtruth_to_pose.py script with the ground truth CSV file. The estimated.txt file is generated during the simulation.
- With the groundtruth.txt and estimated.txt files in the same folder, run the following command to generate error plots:
```
python3 analyze_trajectory_single.py <result_folder>
```
## Live Sensors
To run with a live stereo camera and IMU stream, modify the sensors.yaml file with the appropriate device configurations. Then launch:
```
source devel/setup.bash
roslaunch vio_msckf live.launch
```

## Configuration
The VIO system parameters can be configured through the params.yaml file. This includes settings for the camera and IMU sensor models, as well as tuning parameters for the MSCKF filter.

### Contribution
Contributions are welcome! Please open an issue or submit a pull request for any bugs, improvements, or new features.

## References

Mourikis, A. I., & Roumeliotis, S. I. (2007). A multi-state constraint Kalman filter for vision-aided inertial navigation. In Proceedings 2007 IEEE International Conference on Robotics and Automation (pp. 3565-3572).
Leutenegger, S., Lynen, S., Bosse, M., Siegwart, R., & Furgale, P. (2015). Keyframe-based visual--inertial odometry using nonlinear optimization. The International Journal of Robotics Research, 34(3), 314-334.

