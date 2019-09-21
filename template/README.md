# Template ROS package

This is a template ROS package

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
```
rosrun @project_name@ @project_name@
```

## roslaunch

## rosservice
