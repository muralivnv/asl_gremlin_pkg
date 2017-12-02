#!/bin/bash

robot_name=asl_gremlin1

cyan='\033[0;36m'
light_green='\033[1;32m'
normal='\033[0;m'
yellow='\033[1;33m'
light_purple='\033[0;35m'
light_red='\033[1;31m'

if [ "$#" == "0" ] || [ "$1" == "-h" ]; then
    echo -e "\t${yellow}use flags${normal}"
    echo -e "\t${light_red}-nx${normal}: to set new x_waypoints"
    echo -e "\t${light_red}-ny${normal}: to set new y_waypoints"
    echo -e "\t${light_red}-ax${normal}: to concatenate x_waypoints with this data"
    echo -e "\t${light_red}-ay${normal}: to concatenate y_waypoints with this data\n"
    echo -e "\t${light_red}-f${normal}:  to specify waypoint data from file\n"

    echo -e "${light_green}ex: ./set_waypoints.sh -nx \"20, 10+10*cos(45*pi/180),..\" -ny \"10, 10+10*sin(45*pi/180),..\" ${normal}\n"
    echo -e "${light_green}ex: ./set_waypoints.sh -ax \"20, 10+10*cos(45*pi/180),..\" -ay \"10, 10+10*sin(45*pi/180),..\" ${normal}\n"
    echo -e "${light_green}ex: ./set_waypoints.sh -ax \"20, 10+10*cos(45*pi/180),..\" ${normal}\n"
    echo -e "${light_green}ex: ./set_waypoints.sh -f waypoints_file.dat ${normal}\n"

elif [ "$1" == "-f" ] || [ "$1" == "-file" ]; then

    if [ "$#" == "3" ]; then
        cmd=$(echo "awk '/$3:|$3/ { for (i=2; i <= NF; i++) printf \$i \" \"}' $2")
        ARGS=$(eval "$cmd")
        ARGS=$(echo $ARGS | awk -F\" '{OFS="\""; for (i=2; i<NF; i+=2) gsub(/ /,"", $i); print}')
        rosrun trajectory_generation waypointSet_client __ns:=${robot_name} $ARGS
    else
        cmd=$(echo "awk '\$1 == \"-nx\" || \$1 == \"-ny\" {print \$0}' $2 | tr -s '\n' ' ' ")
        ARGS=$(eval "$cmd")
        rosrun trajectory_generation waypointSet_client __ns:=${robot_name} $ARGS
    fi
else
	ARGS=""
	for var in "$@"
	do
		ARGS=$(echo "$ARGS "$(echo -e "$var" | tr -d '[:space:]' | tr -s ',')"")
	done
    rosrun trajectory_generation waypointSet_client __ns:=${robot_name} $ARGS
fi