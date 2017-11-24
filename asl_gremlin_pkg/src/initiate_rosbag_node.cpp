/**
 * @brief Record data to bag when sim starts node
 * @file initiate_rosbag_node.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <cstdio>
#include <string>
#include <array>

std::string exec_cmd(const std::string& cmd)
{
    std::array<char, 256> buffer;
    std::string cmd_output;

    FILE* pipe= popen(cmd.c_str(), "r");

    if (!pipe)
    { throw std::runtime_error("popen() failed"); }

    while (!feof(pipe))
    {
        if (fgets(buffer.data(), 256, pipe) != nullptr)
        { cmd_output += buffer.data(); }
    }
    pclose(pipe);
    return cmd_output;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_node");
    ros::NodeHandle nh;

    asl_gremlin_pkg::SubscribeTopic<std_msgs::Bool> sim_start(nh,"start_sim");

    double rate = 10.0;
    if (!nh.getParam("sim/rate", rate))
    {
        ROS_WARN("Unable access parameter /%s/sim/rate, setting rate as 10Hz",
                    ros::this_node::getNamespace().c_str());
    }
    ros::Rate loop_rate(rate);

    bool initiated_sim = false;
    
    ROS_INFO("Initialized:= %s",ros::this_node::getName().c_str());

    std::string rosbag_pid_process_num;
    while (ros::ok())
    {
        if ( (sim_start.get_data())->data && !initiated_sim)
        {
            rosbag_pid_process_num = exec_cmd("bash $HOME/asl_gremlin1/src/bash_scripts/record_sim_data.sh");
            ROS_INFO("Initialized:= data recording");
            initiated_sim = true;
        }
        else if (initiated_sim && !(sim_start.get_data())->data )
        {
            ROS_INFO("Aborted:= data recording"); 

            std::string kill_bag_cmd("kill -INT "+rosbag_pid_process_num);
            auto res = std::system(kill_bag_cmd.c_str());
            initiated_sim = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
