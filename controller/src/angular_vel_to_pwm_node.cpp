/**
 * @brief Angular-Vel-To-PWM Node
 * @file angular_vel_to_pwm_node.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#include <ros/ros.h>
#include <controller/OmegaToPWM.h>
#include <asl_gremlin_msgs/MotorPwm.h>
#include <string>

using namespace controller;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "angular_vel_to_pwm");
    ros::NodeHandle w2pwm_nh;
    

    OmegaToPWM omega_to_pwm(w2pwm_nh);

    ros::spinOnce();

    std::string pwm_pub_topic;

    pwm_pub_topic = asl_gremlin_pkg::GetParam_with_shutdown<std::string>
                    (w2pwm_nh,"controller/cmd_pwm_topic", __LINE__); 

    ros::Publisher pwm_pub = w2pwm_nh.advertise<asl_gremlin_msgs::MotorPwm>
                                                        (pwm_pub_topic,20);
    double rate = 10.0;
    if (!w2pwm_nh.getParam("sim/rate", rate))
    {
        ROS_WARN("Unable access parameter /%s/sim/rate, setting rate as 10Hz",
                    ros::this_node::getNamespace().c_str());
    }
    ros::Rate loop_rate(rate);
    
    ROS_INFO("Initialized:= %s",ros::this_node::getName().c_str());
    while(ros::ok())
    {
        pwm_pub.publish(*(omega_to_pwm.convert_omega_to_pwm()));
        ros::spinOnce();
        loop_rate.sleep();
    }

}

