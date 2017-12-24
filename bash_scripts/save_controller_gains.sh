#!/bin/bash

source this_robot_name.sh

robot_name=$ROBOT_NAME

if [ "$robot_name" == "" ]; then
    robot_name="asl_gremlin1"
fi

script_path="$(cd "$(dirname "$0")"; pwd -P)"
pkg_head=$(echo "${script_path//bash_scripts/}")

control_gains_file="${pkg_head}asl_gremlin_pkg/config/tuned_control_gains.yaml"
control_gains_new_file=$(echo "${control_gains_file//tuned_control_gains.yaml/}tuned_control_gains_mod.yaml")

lambda_x_data=$(rosparam get ${robot_name}/backstepping_controller/lambda_x)
lambda_y_data=$(rosparam get ${robot_name}/backstepping_controller/lambda_y)
lambda_theta_data=$(rosparam get ${robot_name}/backstepping_controller/lambda_theta)

awk '{if ($1=="lambda_x:"){$2=lx_data; print}
      else if ($1=="lambda_y:"){$2=ly_data; print}
      else if ($1=="lambda_theta:"){$2=lt_data; print}
      else {print}
     }' lx_data=$lambda_x_data \
        ly_data=$lambda_y_data \
        lt_data=$lambda_theta_data \
        $control_gains_file > $control_gains_new_file

rm -f $control_gains_file

awk 'NR==2,NR==4 {$0="    "$0}{print}' $control_gains_new_file > $control_gains_file

rm -f $control_gains_new_file

echo "saved gains for the controller.."