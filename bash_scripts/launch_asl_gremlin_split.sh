#!/bin/bash

# give the robot a name, so all the nodes will be launched under it's namespace
robot_name=asl_gremlin1

# defining IP address to which ROS-MASTER should register or connect(if already MASTER is running with this IP)
IP=$(hostname -I | awk '{print $1}') # get the IP of the system

# defining port number of Arduino which controls motors and gets encoder ticks
arduino_port_num=/dev/ttyACM0

# Launch the nodes
export ROS_IP=$IP
export ROS_MASTER_URI=http://$IP:11311

session_name="asl_gremlin_session"

if tmux has-session -t "$session_name"; then
    echo "Session $session_name already exits"
    exit 0
fi

tmux new-session -d -s "$session_name" roslaunch asl_gremlin_pkg launch_mavros_rosserial.launch robot_name:=$robot_name \
																								arduino_port:=$arduino_port_num 
sleep 5s

tmux select-window -t "$session_name":0
tmux select-pane -t 0
tmux split-window -v roslaunch asl_gremlin_pkg launch_asl_gremlin_pkg.launch robot_name:=$robot_name

tmux select-window -t "$session_name":0
tmux select-pane -t 0
tmux split-window -h

tmux attach -t "$session_name"