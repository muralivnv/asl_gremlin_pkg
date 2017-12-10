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
    echo -e "\t${light_red}-encoder_cmp${normal}(or) ${light_red}1${normal}: to use ENCODER for pose and COMPASS for heading"
    echo -e "\t${light_red}-gps_cmp${normal}(or) ${light_red}0${normal}: to use GPS for pose and COMPASS for heading"
    echo -e "\t${light_red}-gps${normal} (or) ${light_red}2${normal}: to use only GPS for both pose and heading\n"
    echo -e "${light_green}ex: ./select_feedback -encoder_cmp${normal}\n"
    echo -e "${light_green}ex: ./select_feedback 1${normal}\n"

elif [ "$1" == "-encoder_cmp" ] || [ "$1" == "1" ]; then
    rosrun state_feedback feedbackSelect_client __ns:=${robot_name} 1

elif [ "$1" == "-gps_cmp" ] || [ "$1" == "0" ]; then
    rosrun state_feedback feedbackSelect_client __ns:=${robot_name} 0

elif [ "$1" == "-gps" ] || [ "$1" == "2" ]; then
    rosrun state_feedback feedbackSelect_client __ns:=${robot_name} 2
fi