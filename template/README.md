# @project_name@

ROS node for @project_name@

# 1. Installation

## A. Prerequisities

- [ROS](https://www.ros.org/install/)

## B. Installation

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone https://github.com/ToniRV/@project_name@.git

# Install dependencies from rosinstall file using wstool
wstool init
wstool merge @project_name@/install/@project_name@.rosinstall
wstool update
```

Finally, compile:

```bash
# Compile code
catkin build

# Refresh workspace
source ~/.bashrc
```

# 2. Usage

## rosrun

Cpp:
```bash
rosrun @project_name@ @project_name@
```

Python:
```bash
rosrun @project_name@ @project_name@_script
```

## roslaunch
```bash
roslaunch @project_name@ @project_name@
```

## rosservice
