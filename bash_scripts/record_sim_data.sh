#!/bin/bash

# Obtain the time of launch
launch_time=$(date +%F-%H-%M-%S)

# Create bag_file_name
bag_file_name="asl_gremlin_pkg_${launch_time}.bag"

# run process in background
rosbag record -a -O $HOME/${bag_file_name} > /dev/null &
sleep 2s

# obtain pid process number for the bag_file
# filter rosbag/record (grep) | select pid (print $2) | select first one (NR==1)
pid_num=$(ps aux | grep rosbag/record | awk '{print $2}' | awk 'NR==1')

# send out the file num
echo "${pid_num}"
