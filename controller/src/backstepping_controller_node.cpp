/**
 * @brief Backstepping controller Node
 * @file backstepping_controller_node.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#include <controller/BackSteppingController.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_pkg/GetParam.h>
#include <std_msgs/Bool.h>

#include <thread>
#include <chrono>

using namespace controller;
using namespace asl_gremlin_msgs;
using namespace asl_gremlin_pkg;

int main(int argc, char** argv)
{
    ros::init(argc, argv , "backstepping_controller"); 

    ros::NodeHandle ctrl_nh;
    
    std::string ref_traj_topic_name, act_state_topic;
    ref_traj_topic_name = GetParam_with_shutdown<std::string>
                            (ctrl_nh, "trajectory/publisher_topic", __LINE__);

    act_state_topic = GetParam_with_shutdown<std::string>
                        (ctrl_nh,"state_feedback/feedback_selected", __LINE__); 

    SubscribeTopic<asl_gremlin_msgs::RefTraj> ref_traj(ctrl_nh, ref_traj_topic_name);
    SubscribeTopic<asl_gremlin_msgs::VehicleState> act_state(ctrl_nh, act_state_topic);
  
    
    std::unique_ptr<ControllerBase<RefTraj, VehicleState>> controller = 
                            std::make_unique<BackSteppingController<RefTraj, VehicleState>>(ctrl_nh);
    
    std::string ang_vel_topic;
    ang_vel_topic = GetParam_with_shutdown<std::string>
                        (ctrl_nh, "controller/cmd_angular_vel_topic",__LINE__);

    asl_gremlin_pkg::SubscribeTopic<std_msgs::Bool> sim(ctrl_nh,"start_sim"); 

    ros::Publisher ang_vel_cmd = ctrl_nh.advertise<MotorAngVel>
                                                    (ang_vel_topic, 20);
 
    double rate = 10.0;
    if (!ctrl_nh.getParam("sim/rate", rate))
    {
        ROS_WARN("Unable access parameter /%s/sim/rate, setting rate as 10Hz",
                    ros::this_node::getNamespace().c_str());
    }
    ros::Rate loop_rate(rate);
    ros::spinOnce();

    ROS_INFO("\033[1;32mInitialized\033[0;m:= %s",ros::this_node::getName().c_str());
    bool initialized = false;
    while(ros::ok())
    {
        if ( (sim.get_data())->data )
        {
            if (!initialized)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                ROS_INFO("\033[1;32mInitialized\033[0;m:= generating control commands");
                initialized = true;
            }
            controller->calculate_control_action(*(ref_traj.get_data()),
                                                *(act_state.get_data()));
        }
        else
        { controller->reset(); initialized = false;}

       ang_vel_cmd.publish(*(controller->get_control_action()));

       ros::spinOnce();
       loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}
