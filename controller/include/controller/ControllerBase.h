/**
 * @brief ControllerBase header
 * @file ControllerBase.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _controller_CONTROLLERBASE_H_
#define _controller_CONTROLLERBASE_H_

#include <iostream>
#include <array>
#include <asl_gremlin_msgs/MotorAngVel.h>

namespace controller{


template<typename ref_state_type, typename act_state_type>
class ControllerBase{

    public:
        virtual ~ControllerBase(){}
        virtual void calculate_control_action(const ref_state_type&, const act_state_type&) = 0;

        virtual asl_gremlin_msgs::MotorAngVel* get_control_action() = 0;
        virtual void reset() = 0;
};

} // end namespace {controller}
#endif
