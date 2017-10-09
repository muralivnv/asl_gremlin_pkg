#!/bin/bash

cyan='\033[0;36m'
light_green='\033[1;32m'
normal='\033[0;m'
yellow='\033[1;33m'
light_purple='\033[0;35m'

if [ "$#" == "0" ] || [ "$1" == "-h" ]; then
    echo -e "${yellow}use flags${normal}"
    echo -e "${cyan}-x${normal}: to set x_waypoints"
    echo -e "${cyan}-y${normal}: to set y_waypoints"
    echo -e "Both set of waypoints need to be provided\n"
    echo -e "${light_green}ex: ./set_waypoints.sh -x \"20,10,..\" -y \"10,20,..\" ${normal}\n"
else
    rosrun trajectory_generation waypointSet_client $@
fi

