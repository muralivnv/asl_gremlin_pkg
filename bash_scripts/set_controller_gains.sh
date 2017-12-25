#!/bin/bash

source this_robot_name.sh

# give the robot a name, so all the nodes will be launched under it's namespace
robot_name=$ROBOT_NAME

if [ "$robot_name" == "" ]; then
	robot_name="asl_gremlin1"
fi

cyan='\033[0;36m'
light_green='\033[1;32m'
normal='\033[0;m'
yellow='\033[1;33m'
light_purple='\033[0;35m'
light_red='\033[1;31m'

if [ "$#" == "0" ] || [ "$1" == "-h" ]; then
    echo -e "\t${yellow}use flags${normal}"
    echo -e "\t${light_red}-lx${normal}:   to set lambda_x"
    echo -e "\t${light_red}-ly${normal}:   to set lambda_y"
    echo -e "\t${light_red}-lth${normal}:  to set lambda_theta"
    echo -e "\n${light_green}ex: ./set_controller_gains -lx 0.5 -ly 0.98${normal}\n"
else 
    rosrun controller controllerGainSet_client __ns:=${robot_name} $@
fi