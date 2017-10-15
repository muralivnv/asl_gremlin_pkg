#!/bin/bash
robot_name=asl_gremlin1

cyan='\033[0;36m'
light_green='\033[1;32m'
normal='\033[0;m'
yellow='\033[1;33m'
light_purple='\033[0;35m'


if [ "$#" == "0" ] || [ "$1" == "-h" ]; then
    echo -e "use flags"
    echo -e "${cyan}-encoder_cmp${normal}(or) ${cyan}1${normal}: to use ENCODER for pose and COMPASS for heading"
    echo -e "${cyan}-gps_cmp${normal}(or) ${cyan}0${normal}: to use GPS for pose and COMPASS for heading"
    echo -e "${cyan}-gps${normal} (or) ${cyan}2${normal}: to use only GPS for both pose and heading\n"
    echo -e "${light_green}ex: ./select_feedback -encoder_cmp${normal}\n"
    echo -e "${light_green}ex: ./select_feedback 1${normal}\n"

elif [ "$1" == "-encoder_cmp" ] || [ "$1" == "1" ]; then
    rosrun state_feedback feedbackSelect_client __ns:=${robot_name} 1

elif [ "$1" == "-gps_cmp" ] || [ "$1" == "0" ]; then
    rosrun state_feedback feedbackSelect_client __ns:=${robot_name} 0

elif [ "$1" == "-gps" ] || [ "$1" == "2" ]; then
    rosrun state_feedback feedbackSelect_client __ns:=${robot_name} 2
fi
