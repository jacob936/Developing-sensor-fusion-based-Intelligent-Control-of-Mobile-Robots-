# 🤖 Sensor Fusion-Based Intelligent Control of Mobile Robots

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-green.svg)](https://www.python.org/)

## 📋 Overview

This repository contains the complete source code, implementation, and thesis manuscript for **"Adaptive Sensor Fusion and Sliding Mode Control for Robust Mobile Robot Trajectory Tracking Under Sensing Uncertainty"**.

The project develops an integrated navigation framework combining **Adaptive Extended Kalman Filter (AEKF)** for state estimation with **Adaptive Sliding Mode Controller (ASMC)** for robust trajectory tracking, implemented on the TurtleBot3 platform using ROS2.

---

## 🎯 Introduction

Autonomous mobile robots are increasingly deployed in warehouse, logistics, and industrial applications where reliable navigation and trajectory tracking are essential. However, real-world environments present significant challenges:

- **Sensor noise** from odometry and IMU measurements
- **Model uncertainties** from wheel slippage and mechanical wear
- **Environmental disturbances** from dynamic obstacles

This thesis addresses these challenges through an integrated approach combining adaptive sensor fusion with robust sliding mode control, achieving comparable performance to conventional PID control (RMSE: 1.256m vs 1.261m) with improved robustness to uncertainties.

---

## 🔬 Methodology

🔬 Methodology
This research develops an integrated navigation framework combining three key components:
1. Adaptive Extended Kalman Filter (AEKF)
The AEKF fuses odometry and IMU sensor data to provide accurate state estimation (position, orientation, velocity) at 20 Hz. Unlike conventional EKF with fixed noise parameters, our implementation uses innovation-based adaptive estimation to automatically adjust process and measurement noise covariances in real-time, maintaining accuracy under varying sensor conditions.
2. Adaptive Sliding Mode Controller (ASMC)
The ASMC provides robust trajectory tracking with guaranteed stability. Key features include:

    Gain scheduling that adapts control gains based on tracking error magnitude
    Chattering reduction using boundary layer approximation
    Lyapunov-based stability proof ensuring convergence

3. Reactive Obstacle Avoidance
A LiDAR-based safety layer detects obstacles at 0.8m and navigates around them using arc-motion maneuvers, automatically resuming waypoint tracking once the path is clear.
### Experimental Validation
The framework was implemented in ROS2 and validated in Gazebo simulation using the TurtleBot3 Burger platform. Performance was compared against a well-tuned PID controller under identical conditions (warehouse environment with static obstacles, 9-waypoint perimeter path).

### **Key Components**

#### **1. Adaptive Extended Kalman Filter (AEKF)**

- Fuses odometry and IMU data for accurate state estimation
- Online estimation of process noise (Q) and measurement noise (R) covariances
- Innovation-based adaptive estimation (IAE) algorithm
- Update frequency: 20 Hz

**State Vector:**
x = [x, y, θ, v, ω]ᵀ

**Adaptation Law:**
R̂ = Ĉᵧ - H Pₖ|ₖ₋₁ Hᵀ
Q̂ₖ = Kₖ Ĉᵧ Kₖᵀ

#### **2. Adaptive Sliding Mode Controller (ASMC)**

- Robust trajectory tracking with guaranteed stability
- Gain scheduling based on tracking error magnitude
- Boundary layer approximation for chattering reduction
- Lyapunov-based stability proof
v = v_eq - K_v(t) · sat(s₁/δ)
ω = ω_eq - K_ω(t) · sat(s₂/δ)
**Control Law:**

**Adaptation:**
K̇_v = γ_v|s₁|
K̇_ω = γ_ω|s₂|


#### **3. Obstacle Avoidance**

- Reactive LiDAR-based obstacle detection
- Early detection at 0.8m, avoidance at 0.6m
- Arc-motion navigation around obstacles
- Automatic path re-engagement after clearance

---

## 📁 Project Structure
thesis-repository/
│
├── 📄 README.md                          # This file
├── 📄 LICENSE                            # License file
├── 📄 main.tex                           # Complete thesis manuscript
├──  references.bib                     # Bibliography (60+ references)
│
├── 📁 figures_and_metrics/               # Experimental results
│   ├── 01_asmc_trajectory.png           # ASMC trajectory plot
│   ├── 02_asmc_error.png                # ASMC error plot
│   ├── 03_asmc_control.png              # ASMC control signals
│   ├── 01_pid_trajectory.png            # PID trajectory plot
│   ├── 02_pid_error.png                 # PID error plot
│   ├── 03_pid_control.png               # PID control signals
│   ├── 01_comparison_trajectory.png     # Trajectory comparison
│   ├── 02_comparison_error.png          # Error comparison
│   ├── 03_comparison_control.png        # Control comparison
│   ├── 00_asmc_metrics.txt              # ASMC metrics summary
│   ├── 00_pid_metrics.txt               # PID metrics summary
│   └── 00_comparison_metrics.txt        # Comparison summary
│
├── 📁 chapters/                          # Individual chapter files
│   ├── 01_introduction.tex
│   ├── 02_literature_review.tex
│   ├── 03_methodology.tex
│   └── 04_results_discussion.tex
│
├── 📁 sensor_fusion_ws/                  # ROS2 implementation
│   ├── src/
│   │   └── thesis_control_pkg/
│   │       ├── controller/
│   │       │   ├── asmc_controller_node.py
│   │       │   └── pid_controller_node.py
│   │       ├── sensor_fusion/
│   │       │   └── adaptive_ekf_node.py
│   │       ├── utils/
│   │       │   └── trajectory_generator.py
│   │       └── launch/
│   │           └── warehouse_asmc_launch.py
│   ├── launch/
│   └── CMakeLists.txt
│
└── 📁 documentation/
    ├── thesis_final.pdf                  # Compiled thesis PDF
    └── presentation.pdf                  # Defense presentation
    
    
---

## 📊 Results

### **Quantitative Performance Comparison**

| Metric | ASMC + AEKF | PID + AEKF | Improvement |
|--------|-------------|------------|-------------|
| **RMSE (m)** | 1.256 | 1.261 | +0.4% |
| **Max Error (m)** | 2.146 | 2.130 | -0.7% |
| **Mean Error (m)** | 1.126 | 1.133 | +0.6% |
| **IAE (velocity)** | 31.941 | 29.920 | -6.8% |
| **ISE (velocity)** | 4.746 | 4.488 | -5.8% |
| **Duration (s)** | ~180 | ~180 | - |
| **Samples** | 197 | 197 | - |

### **Key Findings**

✅ **ASMC demonstrates marginally better tracking accuracy** (0.4% improvement in RMSE) compared to well-tuned PID control

✅ **PID demonstrates lower control effort** (6.8% lower IAE), making it more efficient for this specific application

✅ **Both controllers successfully navigate** around static obstacles and complete the waypoint path

✅ **Adaptive EKF provides accurate state estimation** for both control strategies

### **Trajectory Comparison**

![Trajectory Comparison](figures_and_metrics/01_comparison_trajectory.png)

*Figure: Side-by-side comparison of ASMC and PID trajectory tracking performance*

### **Error Comparison**

![Error Comparison](figures_and_metrics/02_comparison_error.png)

*Figure: Tracking error over time for both controllers*

---

## 🛠️ Installation & Usage

### **Prerequisites**

- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- Python 3.10+
- Gazebo Simulator
- TurtleBot3 packages

### **Installation**

```bash
# Clone the repository
git clone https://github.com/jacob-ahemen/developing-sensor-fusion-based-intelligent-control-of-mobile-robots.git
cd thesis-repository

# Install ROS2 dependencies
sudo apt update
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2

# Build the workspace
cd sensor_fusion_ws
colcon build --symlink-install
source install/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Terminal 1: Launch Gazebo
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch ASMC Controller
ros2 launch thesis_control_pkg warehouse_asmc_launch.py

# Terminal 3: Monitor (optional)
ros2 topic echo /state_estimated
