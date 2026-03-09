# EADA Robotics and IoT Course

A hands-on course introducing **ROS 2 (Robot Operating System)** and **IoT concepts** through practical exercises.

## Course Overview

This course teaches the fundamentals of robotics middleware and IoT communication patterns using ROS 2 Jazzy. Students will learn to:
- Build ROS 2 nodes in Python
- Implement publisher/subscriber patterns
- Work with robotics simulation tools (turtlesim)
- Apply IoT concepts like telemetry and dashboards
- Develop distributed systems with message passing

## Prerequisites

- **Python knowledge**: Variables, functions, classes, virtual environments
- **No robotics experience required**: We start from scratch
- **Operating System**: Linux (Ubuntu 24.04 recommended), or Windows/macOS with alternative setup paths

## Course Structure

### [0. Course Preparation](0_course_preparation/0_course_preparation.md)
Setup instructions for three different paths:
- **Path A**: Docker setup (Linux users)
- **Path B**: WSL2 + Native ROS2 (Windows users)
- **Path C**: VM setup (macOS users)

### [1. Class 1 Exercises](1_class_1_exercises/1_class_1_exercises.md)
Introduction to ROS 2 fundamentals:
- **Exercise 1**: Hello World Node
- **Exercise 2**: Publisher/Subscriber pattern
- **Exercise 3**: Turtlesim basics
- **Exercise 4**: Turtlesim control and IoT Dashboard (take-home)

### [2. Class 2 Exercises](2_class_2_exercises/2_class_2_exercises.md)
Advanced ROS 2 concepts:
- **Services**: Creating and calling ROS 2 services
- **TF Broadcasting**: Coordinate frame transforms
- **Actions**: Implementing action servers
- **URDF/Xacro**: Robot description formats

### [3. Class 3 Exercises](3_class_3_exercises/3_class_3_exercises.md)
Simulation and autonomous control concepts:
- **Gazebo fundamentals**: Worlds, models, plugins, and simulation runtime
- **Gazebo transport**: Inspecting topics/services and scene state
- **ROS-Gazebo Integration**: Bridging with `ros_gz_bridge`
- **Model spawning**: Creating entities with `ros_gz_sim/create`

### [4. Class 4 Exercises](4_class_4_exercices/exercises_session4.md)
Sensors, simulation, and autonomous behaviour:
- **Exercise 1**: Add Differential Drive Plugin to the robot URDF
- **Exercise 2**: Add a LiDAR sensor and read minimum distance while moving
- **Exercise 3**: Add an IMU sensor and log orientation and acceleration
- **Exercise 4**: Use launch files and inspect bridged topics
- **Exercise 5**: Add an RGB camera to the robot
- **Exercise 6**: Detect a red object using OpenCV and cv_bridge
- **Exercise 7**: Finite State Machine — take-home exam (due 22nd March)

### [5. Final Project](final_project/FINAL_PROJECT.md)
Capstone project for the course.

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [Docker Documentation](https://docs.docker.com/)
- [WSL Documentation](https://learn.microsoft.com/en-us/windows/wsl/)

---

**Ready to start?** Head to [Course Preparation](0_course_preparation/0_course_preparation.md) to set up your environment!
