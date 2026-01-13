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

**Alternative paths for macOS/Windows users:** If you prefer to stay on macOS or Windows and don't want to dual boot:
+ **Windows users**: See [Path B: WSL2 + Native ROS2](#path-b-wsl2--native-ros2-windows-users) (recommended, uses existing WSL2)
+ **macOS users**: See [Path C: VM Setup](#path-c-vm-setup-macos-users) (uses VirtualBox)

b) Hardware Requirements
+ Minimum 8 GB RAM (12 GB recommended for VM users)
+ Free disk space:
  + For Docker setup: 10 GB minimum
  + For VM setup: 30 GB minimum

1) Install Docker

If you don't have Docker installed, follow the official installation guide for your operating system:

**Linux (Ubuntu/Debian):**
```bash
# Update package index
sudo apt-get update

# Install prerequisites
sudo apt-get install ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up the repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add your user to docker group (to run without sudo)
sudo usermod -aG docker $USER

# Log out and log back in for group changes to take effect
```

**Windows:**
+ Download and install [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)
+ Requires WSL2 backend
+ Follow the installation wizard

**macOS:**
+ Download and install [Docker Desktop for Mac](https://docs.docker.com/desktop/install/mac-install/)
+ Follow the installation wizard

**Verify Docker installation:**
```bash
docker --version
docker run hello-world
```

If you see "Hello from Docker!" message, Docker is installed correctly.

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

6) (Optional) VSCode Installation

Visual Studio Code is a powerful code editor that will make your development experience much better.

**Installation:**

+ **Download VSCode**: [https://code.visualstudio.com/download](https://code.visualstudio.com/download)
+ **Linux Installation Guide**: [https://code.visualstudio.com/docs/setup/linux](https://code.visualstudio.com/docs/setup/linux)
+ **Windows Installation Guide**: [https://code.visualstudio.com/docs/setup/windows](https://code.visualstudio.com/docs/setup/windows)
+ **macOS Installation Guide**: [https://code.visualstudio.com/docs/setup/mac](https://code.visualstudio.com/docs/setup/mac)

**Note:** Instructions on how to use VSCode with Docker for editing ROS2 files are provided in the Class 1 exercises document.

---

# Path B: WSL2 + Native ROS2 (Windows Users)

**Recommended for Windows users** - If you installed Docker Desktop, you already have WSL2! This path installs ROS2 directly in Ubuntu on WSL2.

## Advantages:
+ No Docker containers needed
+ GUI apps work with WSLg (Windows 11) or VcXsrv (Windows 10)
+ Better performance than Docker
+ All commands identical to native Linux

## 1) Verify WSL2 is installed

Open PowerShell or Command Prompt:
```bash
wsl --list --verbose
```

You should see Ubuntu listed. If not, install it:
```bash
wsl --install -d Ubuntu-24.04
```

## 2) Install Ubuntu GUI support

**Windows 11:** WSLg is built-in, no setup needed!

**Windows 10:** Install VcXsrv:
+ Download from [SourceForge VcXsrv](https://sourceforge.net/projects/vcxsrv/)
+ Run XLaunch with default settings
+ In WSL Ubuntu terminal, run:
```bash
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc
source ~/.bashrc
```

## 3) Open Ubuntu in WSL2

Open Windows Terminal or search for "Ubuntu" in Start menu.

## 4) Install ROS2 Jazzy

Inside the Ubuntu terminal:

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop python3-argcomplete python3-colcon-common-extensions -y

# Source ROS2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 5) Install development tools

```bash
# Install terminator
sudo apt install terminator -y
```

If you see the turtle window, you're ready!

## Notes for Class Exercises:
+ Replace `/home/user/exchange` with `~` (your home directory)
+ All ROS2 commands work identically
+ You can edit files with VSCode on Windows using the WSL extension

---

# Path C: VM Setup (macOS Users)

**Recommended for macOS users** - Docker GUI support on macOS is unreliable. A VM provides the best experience.

## 1) Install VirtualBox

+ Download VirtualBox for macOS: [VirtualBox Downloads](https://www.virtualbox.org/wiki/Downloads)
+ Install the `.dmg` file
+ Also download and install the VirtualBox Extension Pack

## 2) Download Ubuntu 24.04 ISO

+ Download: [Ubuntu 24.04 Desktop](https://ubuntu.com/download/desktop)
+ File size: ~5 GB

## 3) Create Virtual Machine

Open VirtualBox and create a new VM:

**VM Settings:**
+ Name: ROS2-Ubuntu
+ Type: Linux
+ Version: Ubuntu (64-bit)
+ Memory: 6144 MB (6 GB, or 8 GB if you have 16+ GB RAM)
+ Create virtual hard disk: VDI, dynamically allocated, 35 GB

**Before starting:**
1. Select VM → Settings
2. **System** → Processor: 2-4 CPUs
3. **Display** → Video Memory: 128 MB, Enable 3D Acceleration
4. **Storage** → Empty → Click disk icon → Choose Ubuntu ISO
5. Click OK

## 4) Install Ubuntu

1. Start the VM
2. Follow Ubuntu installation wizard
3. Choose "Install Ubuntu"
4. Use default settings (erase disk - this only affects the virtual disk)
5. Create username and password
6. After installation, restart VM
7. Remove ISO: Settings → Storage → Remove Ubuntu ISO from optical drive

## 5) Install Guest Additions

Inside Ubuntu VM:
```bash
sudo apt update
sudo apt install build-essential dkms linux-headers-$(uname -r) -y
```

VirtualBox menu: Devices → Insert Guest Additions CD image

Then in terminal:
```bash
cd /media/$USER/VBox_GAs_*
sudo ./VBoxLinuxAdditions.run
sudo reboot
```

This enables clipboard sharing, better resolution, and folder sharing.

## 6) Install ROS2 Jazzy

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop python3-argcomplete python3-colcon-common-extensions -y

# Source ROS2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 7) Install tools

```bash
sudo apt install terminator -y
# VSCode
sudo snap install code --classic
```

## 8) Create workspace

```bash
mkdir -p ~/ros2_course_ws/src
cd ~/ros2_course_ws
colcon build
echo "source ~/ros2_course_ws/install/setup.bash" >> ~/.bashrc
```