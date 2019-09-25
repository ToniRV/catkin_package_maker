#!/usr/bin/env bash

# Store current directory
cur_dir=$(pwd)

# Create a tmp directory
tmp_dir=$(mktemp -d /tmp/catkin_package_maker.XXXXXXXXX)

# Clone repo
#echo "Cloning catkin_package_maker to tmp dir: $tmp_dir"
git clone --quiet https://github.com/ToniRV/catkin_package_maker.git $tmp_dir

# Ask for project name
read -p "Type project name: " project_name

# Make catkin package
cd $tmp_dir
make build project_name=$project_name

# Copy catkin package to wherever this script was executed
cp -r $tmp_dir/$project_name $cur_dir

# Delete the tmp directory
# rm -rf "$tmp_dir"

