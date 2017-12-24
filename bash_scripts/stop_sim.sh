#!/bin/bash

source this_robot_name.sh

# give the robot a name, so all the nodes will be launched under it's namespace
robot_name=$ROBOT_NAME

if [ "$robot_name" == "" ]; then
	robot_name="asl_gremlin1"
fi

stop_motors='rostopic pub --once /'$robot_name'/arduino/cmd_pwm std_msgs/Int16MultiArray "data: [0,0]"'
stop_cmd='rostopic pub --once /'$robot_name'/start_sim std_msgs/Bool "data: false"'

eval $stop_motors
eval $stop_cmd
