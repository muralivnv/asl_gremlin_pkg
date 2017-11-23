#!/bin/bash

rostopic pub --once /asl_gremlin1/arduino/cmd_pwm std_msgs/Int16MultiArray "data: [0,0]"
rostopic pub --once /asl_gremlin1/start_sim std_msgs/Bool "data: false"
