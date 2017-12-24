/**
 * @brief Encoder Count to Angular velocity definitions
 * @file EncoderDataToOmega.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#include <state_feedback/EncoderDataToOmega.h>

using namespace state_feedback;
using namespace asl_gremlin_pkg;

EncoderDataToOmega::EncoderDataToOmega(ros::NodeHandle& nh)
{
    auto node_namespace = ros::this_node::getNamespace().c_str();
    if(!nh.getParam("motor/left_encoder_ticks_per_meter",encoder_left_ticks_per_meter_))
    { 
        ROS_ERROR("Can't access parameter '/%s/motor/left_encoder_ticks_per_meter', shutting down",
                    node_namespace);
        ros::shutdown();
    }
    if(!nh.getParam("motor/right_encoder_ticks_per_meter",encoder_right_ticks_per_meter_))
    { 
        ROS_ERROR("Can't access parameter '/%s/motor/right_encoder_ticks_per_meter', shutting down",
                    node_namespace);
        ros::shutdown();
    }
    if(!nh.getParam("wheel/radius",radius_of_wheel_))
    { 
        ROS_WARN("Can't access parameter '/%s/wheel/radius', setting to default 0.06858m",
                    node_namespace);
        radius_of_wheel_ = 0.06858;
    }

    std::string encoder_left_topic, encoder_right_topic;

    if(!nh.getParam("state_feedback/encoder/left_timeStamped_topic",encoder_left_topic))
    { encoder_left_topic = "state_feedback/encoder_left_timeStamped"; }

    if(!nh.getParam("state_feedback/encoder/right_timeStamped_topic",encoder_right_topic))
    { encoder_right_topic = "state_feedback/encoder_right_timeStamped"; }

    left_wheel_data_  = new asl_gremlin_pkg::SubscribeTopic<std_msgs::Float64MultiArray>(nh, encoder_left_topic, 250);
    right_wheel_data_ = new asl_gremlin_pkg::SubscribeTopic<std_msgs::Float64MultiArray>(nh, encoder_right_topic, 250);

    ros::spinOnce();
    update_encoder_starting_values();
}

EncoderDataToOmega::~EncoderDataToOmega()
{
    delete left_wheel_data_;
    delete right_wheel_data_;
}


void EncoderDataToOmega::update_encoder_starting_values()
{
    if ( !(left_wheel_data_->get_data())->data.empty())
    {
        prev_left_encoder_ticks_ = (left_wheel_data_->get_data())->data[0];
        prev_left_encoder_time_  = (left_wheel_data_->get_data())->data[1];
    }

    if ( !(right_wheel_data_->get_data())->data.empty())
    {
        prev_right_encoder_ticks_ = (right_wheel_data_->get_data())->data[0];
        prev_right_encoder_time_  = (right_wheel_data_->get_data())->data[1];
    }
}

void EncoderDataToOmega::update_delta_ticks()
{
    if ( !(left_wheel_data_->get_data())->data.empty())
    {
        double left_encoder_ticks_now = (left_wheel_data_->get_data())->data[0];
        double left_encoder_time_now =  (left_wheel_data_->get_data())->data[1];

       delta_left_encoder_ticks_ = left_encoder_ticks_now - prev_left_encoder_ticks_;
       delta_left_encoder_time_  = left_encoder_time_now  - prev_left_encoder_time_;

       prev_left_encoder_ticks_ = left_encoder_ticks_now;
       prev_left_encoder_time_ = left_encoder_time_now;
    }

    if ( !(right_wheel_data_->get_data())->data.empty())
    {
        double right_encoder_ticks_now = (right_wheel_data_->get_data())->data[0];
        double right_encoder_time_now =  (right_wheel_data_->get_data())->data[1];

       delta_right_encoder_ticks_ = right_encoder_ticks_now - prev_right_encoder_ticks_;
       delta_right_encoder_time_  = right_encoder_time_now  - prev_right_encoder_time_;

       prev_right_encoder_ticks_ = right_encoder_ticks_now;
       prev_right_encoder_time_ = right_encoder_time_now;
    }
}


void EncoderDataToOmega::calculate_angular_velocities()
{
    ros::spinOnce();
    update_delta_ticks();

    if (delta_left_encoder_time_ != 0.0)
    {
        left_wheel_angular_velocity_ = (delta_left_encoder_ticks_)/
                        (encoder_left_ticks_per_meter_*radius_of_wheel_*delta_left_encoder_time_);
    }

    if (delta_right_encoder_time_ != 0.0)
    {
        right_wheel_angular_velocity_ = (delta_right_encoder_ticks_)/
                        (encoder_right_ticks_per_meter_*radius_of_wheel_*delta_right_encoder_time_);
    }
}


double EncoderDataToOmega::get_left_wheel_angular_vel()
{ return left_wheel_angular_velocity_; }

double EncoderDataToOmega::get_right_wheel_angular_vel()
{ return right_wheel_angular_velocity_; }

