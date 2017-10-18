/**
 * @brief AngularVelocityToPWM header
 * @file OmegaToPWM.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _controller_OMEGATOPWM_H_
#define _controller_OMEGATOPWM_H_

#include <vector>
#include <ros/ros.h>
#include <utility_pkg/custom_algorithms.h>
#include <asl_gremlin_pkg/GetParam.h>
#include <asl_gremlin_msgs/MotorPwm.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>

namespace controller{

class OmegaToPWM{
    
    asl_gremlin_msgs::MotorPwm* pwm_cmd_;

    std::vector<double> pwm_lookup_, omega_lookup_;
    asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::MotorAngVel>* omega_cmd_;

    public:
        OmegaToPWM(ros::NodeHandle&);
        ~OmegaToPWM();
        asl_gremlin_msgs::MotorPwm* convert_omega_to_pwm();
};

} // end namespace {controller}


#endif
