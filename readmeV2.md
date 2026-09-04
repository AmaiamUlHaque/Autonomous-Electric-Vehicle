# Autonomous-Electric-Vehicle
Autonomous Vehicle with Multi-Modal Localization and Mapping

A comprehensive, semester-long engineering project to design, implement, and validate a full-stack autonomous vehicle system. This repository documents the complete development lifecycle—from embedded Linux configuration and ROS integration to sensor fusion, state estimation, and model-predictive control—culminating in a functional autonomous vehicle capable of real-time environment perception and navigation.

<p align="center">
  <img src="./media/lab7_autonomous_driving.gif" alt="Autonomous Driving Demo" width="600"/>
</p>
*The MacAEV navigating autonomously using a virtual barriers algorithm with Quadratic Programming optimization, successfully following a hallway and avoiding obstacles.*

---

## Table of Contents
1. [🎯 Purpose & System Architecture](#-purpose--system-architecture)
2. [🛠️ Hardware Platform & Software Stack](#%EF%B8%8F-hardware-platform--software-stack)
3. [⚙️ Technical Deep Dive: Algorithms & Implementation](#%EF%B8%8F-technical-deep-dive-algorithms--implementation)
    - [Motor Control & Characterization](#motor-control--characterization)
    - [Sensor Integration & Calibration](#sensor-integration--calibration)
    - [State Estimation & Localization](#state-estimation--localization)
    - [Mapping & Environment Perception](#mapping--environment-perception)
    - [Autonomous Navigation & Control](#autonomous-navigation--control)
4. [🏁 Final System Performance](#-final-system-performance)
5. [📚 Technical Skills & Competencies](#-technical-skills--competencies)
6. [🔭 Future Work & Optimization](#-future-work--optimization)
7. [🙏 Acknowledgements](#-acknowledgements)

---

## 🎯 Purpose & System Architecture

The primary objective of this project was to architect and implement a complete autonomous vehicle software stack, from bare-metal motor control to high-level planning and decision-making. This involved integrating a heterogeneous set of sensors and actuators, developing real-time state estimation algorithms, and implementing robust control strategies for navigation in unknown environments.

**System Architecture Overview:**
The vehicle's software architecture follows a modular, ROS-based pipeline:

1.  **Perception Layer:** RPLiDAR (2D laser scanner) and BNO055 IMU provide raw environmental and state data.
2.  **State Estimation Layer:** Fuses IMU and wheel odometry data to provide a robust estimate of the vehicle's pose (`x`, `y`, `yaw`) in the world frame.
3.  **Mapping Layer:** Builds a 2D occupancy grid map of the environment using inverse sensor models and Bayesian filtering.
4.  **Planning & Control Layer:** Implements high-level behaviors (e.g., virtual barriers path planning) and low-level control (feedback-linearized PD control) to generate actuator commands.
5.  **Actuation Layer:** VESC motor controller executes speed and steering angle commands with Field-Oriented Control (FOC).

## 🛠️ Hardware Platform & Software Stack

### Vehicle Platform (MacAEV)
The McMaster AEV is a custom-built, rear-wheel-drive differential-steer platform designed for indoor autonomy research.

| Component | Specifications | Role |
|-----------|----------------|------|
| **Jetson Nano** | ARM Cortex-A57, 4GB RAM, 128-core Maxwell GPU | Main onboard computer for perception, planning, and control. |
| **VESC 6** | STM32F4 MCU, DRV8301 gate driver, 60A continuous current | Sensorless Field-Oriented Control (FOC) of the BLDC motor. |
| **RPLiDAR A2M8** | 360° laser scanner, 720 samples/scan, 12m range, 10Hz rotation | 2D environment perception and obstacle detection. |
| **BNO055 IMU** | 9-DOF (Accel, Gyro, Mag) with internal sensor fusion | Provides absolute orientation (quaternion) for robust odometry. |
| **Logitech F710** | 2.4GHz wireless gamepad | Human-machine interface for manual control and mode switching. |
| **LiPo Battery** | 3S 11.1V, 2200mAh | Primary power source for the entire system. |

### Software Stack & Development Tools
- **Operating System:** Ubuntu 18.04 LTS (on Jetson Nano)
- **Middleware:** ROS Melodic (Robot Operating System) with `catkin` build system
- **Programming Languages:** C++17 (performance-critical nodes), Python 3 (rapid prototyping)
- **Key Libraries & Dependencies:**
    - `vesc_driver`: ROS driver for VESC communication
    - `rplidar_ros`: ROS driver for RPLiDAR
    - `tf2_ros`: ROS coordinate transform library
    - `quadprog`: Quadratic Programming solver for optimization problems
    - `Eigen`: Linear algebra library for C++
    - `OpenCV`: Computer vision library for potential future use
- **Simulation:** F1TENTH Simulator (Gazebo-based)
- **Debugging & Profiling:** `rqt_graph`, `rqt_plot`, `rostopic`, `rosnode`, `rviz`

<p align="center">
  <img src="./media/overview_flowchart.png" alt="Software Pipeline Flowchart" width="700"/>
</p>
*Figure: ROS computation graph showing the modular architecture and data flow between nodes.*

## ⚙️ Technical Deep Dive: Algorithms & Implementation

### Motor Control & Characterization
- **Field-Oriented Control (FOC):** The VESC controller operates in speed mode, using FOC to independently control the `q`-axis (torque-producing) and `d`-axis (flux-producing) currents, with `Id` maintained at zero for maximum torque efficiency.
- **PID Tuning:** The speed controller was tuned using empirical methods (`Kp=0.15`, `Ki=0.10`, `Kd=0.10`) to achieve a balance between rise time, overshoot, and steady-state error. The transfer function of the closed-loop system was analyzed:
  $H_{cl}(s) = \frac{K_p}{s^2 + K_d s + K_p}$
- **Motor Characterization:** Conducted speed and duty cycle sweeps to map the system's response. Collected data on:
    - **RPM vs. Duty Cycle:** Linear relationship with a `speed_to_erpm_gain = 3182.12`.
    - **Current (I_q) vs. Speed:** Inversely proportional relationship due to back-EMF.
    - **Braking Dynamics:** Characterized the `Stop` (open-circuit) vs. `Brake` (short-circuit) behaviors, observing that applying a brake current command (1A to 15A) creates a negative q-axis current, acting as a generator to dissipate kinetic energy.

### Sensor Integration & Calibration
- **Coordinate Frame Management (`tf2`):** Defined and published static transforms between `base_link`, `laser`, and `imu` frames using a consistent `right-hand` rule convention. The transforms were derived from measured physical offsets and rotation matrices:
  $T_{laser}^{base\_link} = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}$
- **IMU Quaternion Processing:** The BNO055 IMU publishes orientation as a quaternion ($q_w, q_x, q_y, q_z$). Converted to a rotation matrix to extract the yaw angle:
  $\theta = \text{atan2}(2(q_w q_z + q_x q_y), 1 - 2(q_y^2 + q_z^2))$
- **LiDAR Data Reprocessing:** Implemented a `laser_callback()` in `experiment.cpp` that rotates the LiDAR data by $180^\circ$ (π radians) to align the RPLiDAR's forward direction with the `base_link` frame. This involved swapping halves of the `ranges` and `intensities` arrays.

### State Estimation & Localization
- **Wheel Odometry:** Implemented a dead-reckoning system using the vehicle's kinematic model (Ackermann steering). The pose update equations were:
  $x_{k+1} = x_k + v_s \cdot \Delta t \cdot \cos(\theta_k)$
  $y_{k+1} = y_k + v_s \cdot \Delta t \cdot \sin(\theta_k)$
  $\theta_{k+1} = \theta_k + \frac{v_s}{l} \cdot \tan(\delta) \cdot \Delta t$
- **IMU-Augmented Odometry:** To mitigate drift from the kinematic model, the IMU yaw measurement replaced the calculated $\theta$ from the kinematic integration. This provides a more accurate and stable heading estimate, essential for long-term navigation.
- **Euler Integration:** Implemented discrete-time integration using a fixed time step ($\Delta t$) from VESC state messages.

### Mapping & Environment Perception
- **Occupancy Grid Mapping:** Developed a Python node implementing the standard occupancy grid mapping algorithm.
- **Inverse Sensor Model:** For each LiDAR scan, cells are classified as:
    - **Occupied:** If the LiDAR beam endpoint falls within the cell ($p_{occ} = 0.7$).
    - **Free:** If the cell lies between the sensor and an obstacle ($p_{free} = 0.3$).
    - **Unknown:** All other cells ($p_{un} = 0.5$).
- **Bayesian Log-Odds Update:** Cell occupancy probabilities are updated recursively using log-odds:
  $l_{k} = l_{k-1} + l_{inv\_sensor} - l_{prior}$
  where $l_{inv\_sensor}$ is the log-odds of the inverse sensor model and $l_{prior}$ is the prior log-odds (usually 0, corresponding to $p=0.5$).
- **Map Parameters:** Configured with a `resolution = 0.05m`, `width = 800`, `height = 800`.

### Autonomous Navigation & Control

#### Line-Following Algorithm
- **Feedback Linearization:** The steering angle command $\delta$ is computed to linearize the nonlinear vehicle dynamics:
  $\delta = \text{atan}\left(\frac{-l}{v_s^2(\cos\alpha_r + \cos\alpha_l)}(-K_p \tilde{d}_{lr} - K_d \dot{d}_{lr})\right)$
  where $\tilde{d}_{lr}$ is the error between desired and actual distances to the left and right walls, and $\alpha_l$, $\alpha_r$ are the angles to the walls.
- **PD Control:** The resulting closed-loop system behaves as a standard second-order system:
  $\ddot{d}_{lr} + K_d \dot{d}_{lr} + K_p d_{lr} = K_p d_{lr}^{des}$
  Allowing for independent tuning of the damping ratio ($\zeta = \frac{K_d}{2\sqrt{K_p}}$) and natural frequency ($\omega_n = \sqrt{K_p}$).

#### Virtual Barriers Algorithm
- **Gap Detection:** Identifies the largest obstacle-free gap in the LiDAR scan within a defined field of view. The vehicle's desired heading is set toward the furthest point in this gap.
- **Quadratic Programming (QP) for Barrier Lines:** Formulates and solves a QP to find two parallel lines that separate the vehicle from obstacles while maximizing the corridor width:
  $\min_{w,s} \frac{1}{2} w^T w$
  subject to:
  $w^T p_i + s \ge 1$ (right-side obstacles)
  $w^T p_j + s \le -1$ (left-side obstacles)
  $-1 + \epsilon \le s \le 1 - \epsilon$
  The distance between the lines is $d = \frac{2}{\sqrt{w^T w}}$, which is maximized by minimizing $w^T w$.
- **Solver:** Used the `quadprog` library, implementing Goldfarb and Idnani's active set method for solving convex QPs.

## 🏁 Final System Performance

The MacAEV successfully demonstrates the following capabilities:

- **Robust Manual Control:** Smooth joystick control via the Logitech F710 with low latency, enabling precise maneuvering.
- **Accurate Localization:** IMU-augmented wheel odometry provides a stable and reliable estimate of vehicle pose, with minimal drift over short distances.
- **Real-Time Mapping:** The occupancy grid mapping node builds a consistent 2D map of the environment, updating at the LiDAR's scan rate (approximately 10Hz).
- **Autonomous Navigation:** The virtual barriers algorithm enables the vehicle to:
    - Successfully navigate hallways with diverse obstacle configurations.
    - Maintain a safe distance from walls and obstacles.
    - React to dynamic changes in the environment (e.g., moving obstacles, though this was not the primary focus).

### Performance Metrics
- **Speed Control:** Achieved a steady-state error of less than 0.1 m/s for speed commands between 0.5 and 1.5 m/s.
- **Steering Control:** Steering angle control within $\pm 0.02$ radians of the commanded angle.
- **Localization Error:** Wheel odometry showed an error of approximately 0.3m over a 5m straight-line path, with the IMU significantly reducing yaw drift.
- **Mapping Accuracy:** Occupancy grid maps accurately represented static obstacles with a resolution of 5cm.

## 📚 Technical Skills & Competencies

This project developed a comprehensive set of technical skills:

### Embedded Systems & Software Development
- **Linux System Administration:** Command-line proficiency in Ubuntu for system management, file operations (`ls`, `cd`, `cp`, `mv`, `grep`, `find`, `sort`), and package management (`apt-get`).
- **ROS (Robot Operating System):** Deep expertise in ROS concepts (nodes, topics, services, messages, launch files) and tools (`rqt_graph`, `rostopic`, `rosnode`, `tf2_ros`).
- **C++/Python Development:** Designed and implemented ROS nodes in C++17 (e.g., `experiment.cpp`) and Python 3 (e.g., `occupancygridmap.py`, `navigation_virtual_barriers.py`).
- **Version Control:** Proficient with Git for code management, collaboration, and change tracking.

### Robotics & Control Theory
- **Field-Oriented Control (FOC):** Understanding of motor control principles and current control loops (d/q axis currents).
- **PID Control:** Tuning and implementation of PID controllers for both speed and steering.
- **Modeling & Simulation:** Use of kinematic models and simulation environments (F1TENTH) for algorithm development and testing.
- **State Estimation:** Implementation of sensor fusion techniques using IMU and wheel odometry.
- **Optimization:** Formulation and solving of quadratic programming problems for path planning and barrier line generation.
- **Occupancy Grid Mapping:** Bayesian filtering methods for building probabilistic maps from sensor data.
- **Kinematics & Dynamics:** Deep understanding of vehicle kinematics (Ackermann steering) and dynamics (kinematic single-track model).

### Engineering & Problem-Solving
- **Hardware-Software Integration:** Interfacing with microcontrollers (VESC), sensors (LiDAR, IMU), and actuators via UART, I2C, and USB.
- **System Integration:** Combining multiple hardware and software components into a cohesive, functioning system.
- **Debugging & Troubleshooting:** Methodical approach to debugging using ROS tools, log files, and data visualization in `rviz`.
- **Technical Documentation:** Creating clear and comprehensive documentation, including system diagrams, code comments, and README files.

## 🔭 Future Work & Optimization

- **Extended Kalman Filter (EKF):** Replace the current dead-reckoning system with a multi-sensor fusion filter (EKF or UKF) integrating wheel odometry, IMU, and potentially GPS or visual odometry for more robust localization.
- **Global Path Planning:** Implement algorithms like A* or RRT on the occupancy grid for navigating to specific target points in a mapped environment.
- **Dynamic Obstacle Handling:** Modify the occupancy grid mapping and planning algorithms to handle and track moving objects.
- **SLAM (Simultaneous Localization and Mapping):** Integrate a full SLAM algorithm (e.g., GMapping, Cartographer) to build maps without manual initialization.
- **Depth Camera Integration:** Use the RealSense RGB-D camera to enhance obstacle detection, particularly for low-lying objects outside the LiDAR plane.

## 🙏 Acknowledgements
- **Dr. Berker Bilgin** for his guidance and support throughout this challenging project.
- **Teaching Assistants:** A special thank you to the TAs for their invaluable technical assistance, patience, and dedication.
- **Lab Partners:** Aayan Siddiqui and Jasmine Smith for their exceptional collaboration, hard work, and problem-solving skills.
- **F1TENTH Community:** For the excellent open-source simulator and packages that served as a foundation for our work.

---
*Built with ❤️ for the pursuit of knowledge and the challenge of building a truly autonomous vehicle from the ground up.*