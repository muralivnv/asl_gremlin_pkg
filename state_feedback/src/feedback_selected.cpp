/**
 * @brief publish selected feedback node
 * @file feedback_selected.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <state_feedback/FeedbackSelected.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feedback_selected");

    ros::NodeHandle feedback_nh;

    state_feedback::FeedbackSelected<3> feedback_method(feedback_nh);

    double rate = 10.0;

    if (!feedback_nh.getParam("sim/rate", rate))
    { ROS_WARN("Unable access parameter /%s/sim/rate, setting rate as 10Hz",
                ros::this_node::getNamespace().c_str()); }

    ros::Rate loop_rate(rate);

    
    ROS_INFO("Initialized:= %s",ros::this_node::getName().c_str());
    while(ros::ok())
    {
        feedback_method.get_gps_data();
        feedback_method.get_enco_data();
        feedback_method.publish();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
