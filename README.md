# ROS PACKAGE MAKER

Automatic generator of ROS packages with the minimal skeleton for development of a ROS node (c++). This is an extremely simplified and cleaner version of `catkin_create_pkg`.

# 1. Installation

## A. Prerequisities

- [ROS](https://www.ros.org/install/).

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
git clone https://github.com/ToniRV/ros_node_template.git

# Install dependencies from rosinstall file using wstool
wstool init
wstool merge ros_node_template/template/install/template.rosinstall
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

## Build the catkin project

Easiest way:
```
make build
```

It will build a template catkin package named `project_name`:

You can use rosrun to actualy run it:
```
rosrun project_name project_name
```

Alternatively, build the catkin package with a name of your choice `PROJECT_NAME`:
```
python make_package.py --project_name PROJECT_NAME
```

Then, run it:
```
rosrun PROJECT_NAME PROJECT_NAME
```

