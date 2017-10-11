#!/bin/bash

rostopic pub --once /asl_gremlin1/start_sim std_msgs/Bool "data: true"
