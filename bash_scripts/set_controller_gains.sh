#!/bin/bash
cyan='\033[0;36m'
light_green='\033[1;32m'
normal='\033[0;m'
yellow='\033[1;33m'
light_purple='\033[0;35m'

if [ "$#" == "0" ] || [ "$1" == "-h" ]; then
    echo -e "${cyan}use flags${normal}\n${light_green}-lx${normal}:   ${light_purple}to set lambda_x${normal}"
    echo -e  "${light_green}-ly${normal}:   ${light_purple}to set lambda_y${normal}"
    echo -e "${light_green}-lth${normal}:  ${light_purple}to set lambda_theta${normal}"
    echo -e "\n${yellow}ex: ./set_controller_gains -lx 0.5 -ly 0.98${normal}\n"
else 
    rosrun controller controllerGainSet_client $@
fi

