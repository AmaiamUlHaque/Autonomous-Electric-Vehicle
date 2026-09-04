# Autonomous-Electric-Vehicle
Autonomous Vehicle with Multi-Modal Localization and Mapping

A comprehensive, semester-long engineering project to build, program, and test a fully autonomous vehicle from the ground up. This repository documents the complete journey—from foundational software and hardware concepts to the implementation of advanced self-driving algorithms—culminating in a functional autonomous vehicle capable of navigating real-world environments.

<p align="center">
  <img src="./media/lab7_autonomous_driving.gif" alt="Autonomous Driving Demo" width="600"/>
</p>
*The MacAEV navigating autonomously using a virtual barriers algorithm, successfully following a hallway and avoiding obstacles.*

---

## Table of Contents
1. [🎯 Purpose & Vision](#-purpose--vision)
2. [🛠️ The System: Hardware & Software](#%EF%B8%8F-the-system-hardware--software)
3. [⚙️ The Process: From Concept to Autonomy](#%EF%B8%8F-the-process-from-concept-to-autonomy)
4. [🏁 Final Results & Performance](#-final-results--performance)
5. [📚 Skills & Competencies Acquired](#-skills--competencies-acquired)
6. [🔭 Future Enhancements](#-future-enhancements)
7. [🙏 Acknowledgements](#-acknowledgements)

---

## 🎯 Purpose & Vision

The primary goal of this project was to design, build, and program a small-scale autonomous electric vehicle (AEV) from the ground up. This endeavor simulated a real-world engineering development cycle, taking a project from initial concept through to a working prototype.

**The Vision:**
To create an AEV that can independently perceive its environment, determine its location, and make intelligent decisions to navigate safely and effectively. This required mastering the entire robotics software pipeline: from low-level motor control to high-level computer vision and planning.

**Key Objectives:**
- **Build a Robust Platform:** Assemble and calibrate the MacAEV, integrating a Jetson Nano, VESC motor controller, RPLiDAR, and an IMU.
- **Master the Software Stack:** Develop proficiency in Linux, ROS (Robot Operating System), and embedded C++/Python.
- **Implement Core Robotics Algorithms:** Program systems for localization (wheel odometry, IMU fusion), mapping (occupancy grid mapping), and autonomous control (wall-following, virtual barriers).
- **Achieve Full Autonomy:** Enable the vehicle to navigate unknown environments safely and reliably using only its onboard sensors and control algorithms.

## 🛠️ The System: Hardware & Software

### Vehicle Platform (MacAEV)
The McMaster Autonomous Electric Vehicle is a custom-built, rear-wheel-drive platform.

| Component | Specifications | Role |
|-----------|----------------|------|
| **Jetson Nano** | ARM Cortex-A57, 4GB RAM | Main onboard computer for high-level processing and decision-making. |
| **VESC 6** | High-performance motor controller | Controls the BLDC motor in speed, current, and duty cycle modes. |
| **RPLiDAR A2M8** | 360° LiDAR, 720 samples/scan | Environment perception, mapping, and obstacle detection. |
| **BNO055 IMU** | 9-DOF (Accel, Gyro, Mag) | Provides absolute orientation (yaw) for robust odometry. |
| **Logitech F710** | Wireless gamepad | Manual control input and mode switching (manual/autonomous/emergency). |
| **LiPo Battery** | 3S 11.1V | Power source for the entire vehicle. |

### Software Stack & Tools
- **OS:** Ubuntu 18.04 (on Jetson Nano)
- **Framework:** ROS Melodic (Robot Operating System)
- **Languages:** C++ (performance-critical nodes), Python (rapid prototyping)
- **Key Libraries:** VESC Tool (motor tuning), OpenCV, Eigen (math), TF2 (coordinate transforms)
- **Simulation:** F1TENTH Simulator (Gazebo-based)

<p align="center">
  <img src="./media/overview_flowchart.png" alt="Software Pipeline Flowchart" width="700"/>
</p>
*Figure: Simplified overview of the software pipeline, showing data flow from sensors to the motor.*

## ⚙️ The Process: From Concept to Autonomy

The project was structured to build a complete robotics stack, where each phase laid the foundation for the next.

### Phase 1: Foundation & Fundamentals

Before we could even think about autonomous driving, we had to build a deep understanding of the tools and platforms we were using.

- **Linux & Embedded Systems:** Gained proficiency in the Linux environment (Ubuntu) through hands-on terminal use. This was essential for navigating the Jetson Nano, managing files, installing software (`sudo apt-get`), and scripting. This knowledge is the bedrock of all embedded systems development.
- **Robot Operating System (ROS):** Introduced to the core concepts of ROS: Nodes, Topics, Services, and the Master. We learned how different parts of a robot's software communicate, setting up our first talker/listener and understanding the computation graph.

### Phase 2: Building the Vehicle's Nervous System

With the software foundation in place, we turned our attention to the vehicle's sensors and actuators—the "nervous system" of the robot.

- **Sensor & Actuator Integration:** We integrated the RPLiDAR, which allowed the vehicle to "see" its environment, and the BNO055 IMU, which allowed it to know its orientation. We also established communication with the VESC motor controller, enabling us to command the motor.
- **Motor Characterization & PID Tuning:** Using the VESC Tool, we characterized the motor's response and tuned its PID speed controller. Understanding how the motor behaves under different control modes (speed, torque, duty cycle) was crucial for smooth and predictable movement.
- **Manual Control:** We configured a wireless gamepad (Logitech F710) to manually drive the vehicle, giving us a reliable way to test and debug the system.

### Phase 3: From Manual to Autonomous: The Brains of the Operation

This was the core phase where we gave the vehicle its "brains," enabling it to operate autonomously.

- **Simulation:** Before running code on the expensive hardware, we used the F1TENTH simulator. This allowed us to rapidly develop and debug algorithms for control, localization, and mapping in a safe, virtual environment.
- **Localization (Wheel Odometry):** We implemented a `dead-reckoning` system that combines VESC-reported speed and IMU yaw data to estimate the vehicle's position (`x`, `y`, `yaw`) over time. This is the vehicle's internal sense of "where it is."
- **Mapping (Occupancy Grid):** We developed a node that uses LiDAR data and the pose from wheel odometry to build a 2D occupancy grid map of the environment. This map provides a representation of static obstacles, essential for navigation.
- **Control Algorithms:** We designed and implemented two advanced control algorithms:
    1.  **Wall-Following:** A feedback-linearized PD controller that keeps the vehicle centered in a hallway.
    2.  **Virtual Barriers:** A more robust algorithm that uses a Quadratic Program to find the largest safe corridor to drive through, avoiding obstacles and navigating complex environments.

## 🏁 Final Results & Performance

The project successfully achieved its major milestones, culminating in a fully functional autonomous vehicle.

**The MacAEV is capable of:**
- **Manual Operation:** Being driven manually via a wireless gamepad for testing and demonstration.
- **Self-Localization:** Estimating its position in a known environment using wheel odometry, corrected by IMU data for orientation.
- **Real-time Mapping:** Generating a 2D map of its surroundings using onboard LiDAR and wheel odometry data.
- **Full Autonomy:** Navigating hallways in a self-driving mode, using the robust virtual barriers algorithm to:
    - Follow the center of the corridor.
    - Detect and avoid static obstacles (like boxes or walls).
    - Find the safest path forward in complex scenarios.

### Demonstration & Visual Evidence
The success of the project is best illustrated by the final demonstration. The following visualization shows the MacAEV in action, autonomously navigating a hallway while successfully avoiding a series of obstacles.

<p align="center">
  <img src="./media/lab7_autonomous_driving.gif" alt="Autonomous Driving Demo" width="600"/>
</p>

## 📚 Skills & Competencies Acquired

This project was a comprehensive learning experience that developed a wide range of technical and professional skills.

### Technical Skills
- **Linux System Administration:** Mastered command-line operations, package management, and system configuration in an Ubuntu environment.
- **ROS Development:** Deep understanding of ROS nodes, topics, services, and the TF framework, gained through debugging and building a complex distributed system.
- **Embedded Systems:** Gained hands-on experience with embedded Linux (Jetson Nano), motor controllers (VESC), and various sensors (LiDAR, IMU).
- **Motor Control:** Acquired practical knowledge of Field-Oriented Control (FOC), PID tuning, and motor characterization.
- **Control Systems:** Designed and tuned feedback controllers (PD, feedback-linearization) for real-world applications.
- **C++ & Python Programming:** Developed performance-critical nodes in C++ and rapid-prototyping nodes in Python.
- **Algorithms:** Implemented foundational robotics algorithms: Wheel Odometry, Occupancy Grid Mapping, and Quadratic Programming for path planning.
- **Simulation:** Used and extended the F1TENTH simulator for testing control algorithms, demonstrating an understanding of the sim-to-real transfer process.

### Software Engineering & Project Management
- **Version Control:** Managed the project codebase using Git, enabling collaboration and change tracking.
- **System Integration:** Successfully integrated multiple hardware and software components into a single, cohesive system.
- **Debugging:** Developed a methodological approach to debugging complex systems using ROS tools (`rqt_graph`, `rostopic`, `rqt_plot`).
- **Documentation:** Created detailed technical documentation for each component and algorithm, as demonstrated in this README.

## 🔭 Future Enhancements
- **Localization:** Integrate an Extended Kalman Filter (EKF) to fuse wheel odometry, IMU, and GPS for more robust and accurate localization.
- **Path Planning:** Implement a global path planner (e.g., A*, RRT) for navigating to specific coordinates.
- **Automated Parking:** Develop a parking algorithm for the vehicle to self-park in a designated spot.
- **Dynamic Obstacle Avoidance:** Modify the mapping algorithm to distinguish between static and dynamic obstacles.
- **Enhanced Perception:** Explore the use of the RGB-D camera for detecting obstacles not in the LiDAR plane.

## 🙏 Acknowledgements
- **Dr. Berker Bilgin** for his invaluable guidance and support throughout the course.
- **Teaching Assistants:** A special thank you to the TAs for their endless patience, willingness to help debug our code, and for keeping the lab running smoothly during late-night sessions.
- **Lab Partners:** Aayan Siddiqui and Jasmine Smith for their collaboration, hard work, and dedication to the project.
- **F1TENTH Community:** For the excellent simulator and open-source packages that served as a starting point for our work.

---
*Built with ❤️ for the challenge of building a truly autonomous vehicle from the ground up.*