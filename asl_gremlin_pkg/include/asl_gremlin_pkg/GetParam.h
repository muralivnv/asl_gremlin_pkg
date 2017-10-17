/**
 * @brief GetParam function header
 * @file GetParam.h
 * @author Murali VNV <muralivnv@gmail.com>
 */

/*
 * Copyright 2017.
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#ifndef _asl_gremlin_pkg_GETPARAM_H_
#define _asl_gremlin_pkg_GETPARAM_H_

#include <string>
#include <ros/ros.h>

namespace asl_gremlin_pkg{

void throw_warn(const std::string& , int);
void throw_error_and_shutdown(const std::string&, int);


template<typename T>
T GetParam_with_shutdown(ros::NodeHandle& nh, 
                        const std::string& const_param_name, 
                        int line_num)
{
    auto param_name = const_param_name;

    if (param_name[0] == '/')
    { param_name.erase(0,1); }

    if (param_name[0] == '/')
    { param_name.erase(0,1); }


    T param_value;
    if(!nh.getParam(param_name, param_value))
    { asl_gremlin_pkg::throw_error_and_shutdown(param_name, line_num); }
    else
    { return param_value; }
}

template<typename T>
T GetParam_with_warn(ros::NodeHandle& nh, const std::string& const_param_name, int line_num)
{
    auto param_name = const_param_name;
    if (param_name[0] == '/')
    { param_name.erase(0,1); }

    if (param_name[0] == '/')
    { param_name.erase(0,1); }

    T param_value;
    if(!nh.getParam(param_name, param_value))
    { asl_gremlin_pkg::throw_warn(param_name, line_num); }
    else
    { return param_value; }
}

} // end namespace {asl_gremlin_pkg}



#endif
