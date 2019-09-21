#!/usr/bin/env bash

# Store current directory
cur_dir=$(pwd)

# Create a tmp directory
tmp_dir=$(mktemp -d /tmp/ros_node_template.XXXXXXXXX)

# Clone repo
echo "Cloning ros_node_template to tmp dir: $tmp_dir"
git clone https://github.com/ToniRV/ros_node_template.git $tmp_dir

# Ask for project name
read -p "Type project name: " project_name.

# Make catkin package
cd $tmp_dir
make build project_name=$project_name

# Copy catkin package to wherever this script was executed
cp -r $tmp_dir/$project_name $cur_dir

# Delete the tmp directory
# rm -rf "$tmp_dir"

