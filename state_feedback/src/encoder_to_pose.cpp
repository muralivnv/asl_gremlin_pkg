/**
 * @brief Actual angular velocity to pose of rover node
 * @file encoder_to_pose.cpp
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
#include <geometry_msgs/PointStamped.h>
#include <state_feedback/ForwardEuler.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <asl_gremlin_pkg/GetParam.h>
#include <utility_pkg/utilities.h>

#include <array>
#include <cmath>
#include <string>

#define deg2rad M_PI/180.0

using namespace state_feedback::numerical_diff;
using namespace state_feedback;

struct roverParam{
    double wl = 0.0, wr = 0.0;
    double r  = 0.06858, b = 0.3353;
};

std::array<double, 3> rover_kinematics(double time,
                                       const std::array<double, 3>& states,
                                        roverParam* params)
{
    double x_dot = (params->wl + params->wr)*0.5*params->r*std::cos(states[2]);
    double y_dot = (params->wl + params->wr)*0.5*params->r*std::sin(states[2]);
    double theta_dot = (params->r/params->b)*(params->wr - params->wl);

    return {x_dot, y_dot, theta_dot};
}


int main(int argc, char** argv)
{
    ros::init(argc, argv,"encoder_to_pose");

    ros::NodeHandle enco2w_nh;

    std::string encoder_pub_name, ang_vel_topic;
    encoder_pub_name = asl_gremlin_pkg::GetParam_with_shutdown<std::string>
                        (enco2w_nh, "state_feedback/encoder/pose_topic", __LINE__);

    ang_vel_topic = asl_gremlin_pkg::GetParam_with_shutdown<std::string>
                        (enco2w_nh, "state_feedback/encoder/ang_vel_topic", __LINE__);

    ros::Publisher encoder_data_pub = enco2w_nh.advertise<geometry_msgs::PointStamped>(encoder_pub_name,10);

    asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::MotorAngVel> actual_angular_vel(enco2w_nh, ang_vel_topic);
    asl_gremlin_pkg::SubscribeTopic<std_msgs::Float64> compass_hdg(enco2w_nh, "mavros/global_position/compass_hdg");
    asl_gremlin_pkg::SubscribeTopic<std_msgs::Bool> sim(enco2w_nh, "start_sim");

    roverParam params;
    geometry_msgs::PointStamped encoder_pose;

    double rate = 10.0;
    if (!enco2w_nh.getParam("sim/rate", rate))
    {
        ROS_WARN("Unable access parameter /%s/sim/rate, setting rate as 10Hz",
                    ros::this_node::getNamespace().c_str());
    }
    ros::Rate loop_rate(rate);
    
    forward_euler_step_size = 1/rate;
    
    std::array<double ,3> integrated_states;
    std::array<double, 3> initial_states{0,0,0};

    ros::spinOnce();

    initial_states[2] = utility_pkg::compass_angle_to_polar_angle((compass_hdg.get_data())->data) * deg2rad;
    double t_initial = 0.0;
    double t_final = t_initial + forward_euler_step_size;

    int msg_count = 0;
    encoder_pose.point.x = initial_states[0];
    encoder_pose.point.y = initial_states[1];
    encoder_pose.point.z = 0;
    encoder_pose.header.seq = msg_count;
    encoder_pose.header.stamp = ros::Time::now();
    encoder_pose.header.frame_id = "local_tangent_enu";

    encoder_data_pub.publish(encoder_pose);
    asl_gremlin_msgs::MotorAngVel* actual_omega;

    bool initiated = false;


    ROS_INFO("\033[1;32mInitialized\033[0;m:= %s",ros::this_node::getName().c_str());
    while(ros::ok())
    {
        if ( (sim.get_data())->data && !initiated)
        {
            ROS_INFO("\033[1;32mStarted\033[0;m:= Encoder based dead-reckoning");
            initial_states[0] = 0.0; initial_states[1] = 0.0;
            initiated = true;
        }
        if (initiated && !(sim.get_data())->data)
        { 
            ROS_INFO("\033[1;31mStopped\033[0;m:= Encoder based dead-reckoning");
            initiated = false; 
        }

        actual_omega = actual_angular_vel.get_data();
        params.wl = actual_omega->wl;
        params.wr = actual_omega->wr;

        initial_states[2] = utility_pkg::compass_angle_to_polar_angle((compass_hdg.get_data())->data) * deg2rad;

        integrated_states = forwardEuler_integration(   rover_kinematics,
                                                        initial_states,
                                                        t_initial, t_final, 
                                                        &params);
        ++msg_count;
        encoder_pose.point.x = integrated_states[0];
        encoder_pose.point.y = integrated_states[1];
        encoder_pose.point.z = 0;
        encoder_pose.header.seq = msg_count;
        encoder_pose.header.stamp = ros::Time::now();
        encoder_data_pub.publish(encoder_pose);

        ros::spinOnce();
        loop_rate.sleep();

        initial_states = integrated_states;
        t_initial = t_final;
        t_final += forward_euler_step_size;
    }
}
