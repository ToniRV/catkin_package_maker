# CATKIN PACKAGE MAKER

Automatic generator of ROS packages with the minimal skeleton for development of a ROS node (c++). This is an extremely simplified and cleaner version of `catkin_create_pkg`.

## 1. Installation

None

## 2. Usage

**Tired of creating catkin packages from scratch?**

Just run: 

```
bash -c "$(curl -sLf https://raw.githubusercontent.com/ToniRV/catkin_package_maker/master/make_catkin_pkg.sh)"
```

And enter the name of your catkin package:

```bash
Type project name:
test
```

Done! Fresh new catkin simple package.

## 3. What's going on?

This will generate a fresh catkin simple package, including filesystem, and even README.md, all updated with the name of your project.

```bash
Creating new project at: ./test
Created file test/package.xml
Created file test/CMakeLists.txt
Created file test/src/test.cpp
Created file test/launch/test.launch
Created file test/README.md
```

> The actual template for the catkin package is in [here](./template). Where all strings between @ will be updated according to user input.

If you have executed this inside a catkin workspace, just build and run:

```
catkin build
# Source your catkin workspace before!
rosrun test test
```

You should see something like:
```
[ INFO] [1569099086.150568624]: Hello World
```

If the command above is too long, just create a bash **function** in your `~/.bashrc`: 

```bash
make_catkin_pkg() {
  bash -c "$(curl -sLf https://raw.githubusercontent.com/ToniRV/catkin_package_maker/master/make_catkin_pkg.sh)"
}
```

After sourcing your `source ~/.bashrc`:
```
make_catkin_pkg
```

Enjoy!

More info [here](http://www.mit.edu/~arosinol/2019/09/21/ROS_Catkin_Package_Maker/).
