## Course preparation for Internet of Things and Robotics

Before starting the course, students should meet the following requirements.

a) Software and System Requirements
+ Basic knowledge of Python (variables, functions, classes, and virtual environments).
+ No prior ROS, robotics or Internet of Things (Iot) experience is required.
+ Docker is installed and working.
+ Operating System:
    + Linux (highly recommended): Ubuntu 24.04 is the best for ROS2 Jazzy.
    + Windows: running ROS2 in Windows is tricky.
        + Possible using WSL2 (Windows Subsystem for Linux) with Ubuntu.
        + We recommend a dual boot configuration with Ubuntu 24.04.
    + macOS: Running ROS2 with macOS is even trickier. While Docker-based setups may partially work, GUI tools (RViz, Gazebo) are often     unstable or unavailable. 
        + We recommend a dual boot configuration with Ubuntu 24.04
b) Hardware Requirements
+ Minimum 8 GB RAM
+ Free disk space

2) Download Docker Image

Open a terminal and run:

```bash
docker pull osrf/ros:jazzy-desktop
```

3) Prepare permissions and create shared folder

Run the following commands on the host machine:
```bash
xhost +
mkdir -p ros2-course-exchange
```

Note: The xhost + command temporarily disables X security to allow GUI applications from Docker. This is acceptable for a course environment.

4) Run the docker container

From the directory where you want the shared folder, run:

```bash
docker run -it --name ros2jazzycourse -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e PULSE_SERVER=$PULSE_SERVER -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd)/ros2-course-exchange:/home/user/exchange --net=host osrf/ros:jazzy-desktop bash
```

This configuration enables:

+ Graphical applications (RViz, rqt, Gazebo)
+ Audio support
+ File sharing via the ~/exchange folder
+ ROS networking without additional setup

5) Install terminator (inside the container)

Once inside the Docker container, run:
```bash
apt update
apt install -y terminator
```
Terminator is a terminal emulator that allows:
+ Splitting the terminal into multiple panes (horizontal and vertical)
+ Running multiple ROS nodes, tools, and commands simultaneously
+ Monitoring logs while launching nodes
+ A more efficient workflow during ROS development and debugging
