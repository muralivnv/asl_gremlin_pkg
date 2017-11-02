/**
 * @brief Encoder Count to Angular velocity header
 * @file EncoderDataToOmega.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#ifndef _state_feedback_ENCODERDATATOOMEGA_H_
#define _state_feedback_ENCODERDATATOOMEGA_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_pkg/GetParam.h>
#include <string>

namespace state_feedback{

class EncoderDataToOmega{
    double left_wheel_angular_velocity_ = 0.0,
            right_wheel_angular_velocity_ = 0.0;

    double encoder_left_ticks_per_meter_ = 0.0,
           encoder_right_ticks_per_meter_ = 0.0;
    
    double prev_left_encoder_time_ = 0.0,
           prev_right_encoder_time_ = 0.0;

    double prev_left_encoder_ticks_ = 0.0,
           prev_right_encoder_ticks_ = 0.0;

    double delta_left_encoder_ticks_ = 0.0,
           delta_right_encoder_ticks_ = 0.0;

    double delta_left_encoder_time_ = 0.0,
           delta_right_encoder_time_ = 0.0;
    
    double radius_of_wheel_ = 0.06858;
 
    asl_gremlin_pkg::SubscribeTopic<std_msgs::Float64MultiArray>* left_wheel_data_;
    asl_gremlin_pkg::SubscribeTopic<std_msgs::Float64MultiArray>* right_wheel_data_;

    public:
        EncoderDataToOmega(ros::NodeHandle&);
        ~EncoderDataToOmega();

        void calculate_angular_velocities();
        double get_left_wheel_angular_vel();
        double get_right_wheel_angular_vel();

        void update_encoder_starting_values();
        void update_delta_ticks();

};

} // end namespace {state_feedback}
#endif
