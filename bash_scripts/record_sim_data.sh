#!/bin/bash

source this_robot_name.sh

# give the robot a name, so all the nodes will be launched under it's namespace
robot_name=$ROBOT_NAME

if [ "$robot_name" == "" ]; then
	robot_name="asl_gremlin1"
fi

# Obtain the time of launch
launch_time=$(date +%F-%H-%M-%S)

# Create bag_file_name
bag_file_name="${robot_name}_${launch_time}.bag"

# run process in background
rosbag record -a -O $HOME/${bag_file_name} > /dev/null &
sleep 2s

# obtain pid process number for the bag_file
# filter rosbag/record (grep) | select pid (print $2) | select first one (NR==1)
pid_num=$(ps aux | grep rosbag/record | awk 'NR==1{print $2}')

# send out the file num
echo "${pid_num}"
