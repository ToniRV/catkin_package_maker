# CATKIN PACKAGE MAKER

Automatic generator of ROS packages with the minimal skeleton for development of a ROS node (c++). This is an extremely simplified and cleaner version of `catkin_create_pkg`.

It makes use of [catkin simple](https://github.com/catkin/catkin_simple) to simplify the verbose `CMakeLists.txt` and `package.xml` that `catkin_create_pkg` generates, and builds a package directory updated with the project name provided by the user.

## 1. Installation

## A. Prerequisities

- [ROS](https://www.ros.org/install/)

## B. Installation

As usual, setup your catkin workspace if you don't have one:

```bash
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
```

In your catkin workspace, clone catkin simple:

```bash
# Clone catkin simple
cd src
git clone https://github.com/catkin/catkin_simple
```

## 2. Usage

**Tired of creating catkin packages from scratch?**

Just run, in your catkin workspace:

```bash
cd ~/catkin_ws/src
bash -c "$(curl -sLf https://raw.githubusercontent.com/ToniRV/catkin_package_maker/master/make_catkin_pkg.sh)"
```

And enter the name of your catkin package:

```bash
Type project name:
foo
```

Done! Fresh new catkin simple package.

Now, compile the code:

```bash
# Compile code
catkin build foo

# Refresh workspace
source ~/catkin_ws/src/devel/setup.sh
```

Run to test that it worked:
```bash
rosrun foo foo
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

After sourcing your bashrc `source ~/.bashrc`, you can start creating catkin packages in a split of a second using:
```
make_catkin_pkg
```

Enjoy!

## 3. What's going on?

This will generate a fresh catkin simple package, including filesystem, and even README.md, all updated with the name of your project.

```bash
Creating new project at: ./foo
Created file foo/package.xml
Created file foo/CMakeLists.txt
Created file foo/src/foo.cpp
Created file foo/launch/foo.launch
Created file foo/README.md
```

> The actual template for the catkin package is in [here](./template). Where all strings between @ will be updated according to user input.

If you have executed this inside a catkin workspace, just build and run:

```bash
catkin build

# Source your catkin workspace before!
# Change `foo` for your actual package name
rosrun foo foo
```

More info [here](http://www.mit.edu/~arosinol/2019/09/21/ROS_Catkin_Package_Maker/).
