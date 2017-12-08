/**
 * @brief system utilities header
 * @file utilities.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _utility_pkg_UTILITIES_H_
#define _utility_pkg_UTILITIES_H_

#include <cmath>
#include <string>
#include <cstdlib>
#include <ros/ros.h>

namespace utility_pkg{

// function which converts the compass angle (0 to 360) NED to
// (-180 to 180) ENU frame
double compass_angle_to_polar_angle(double);

double wrapTo2Pi(double);

void stop_rover(const std::string&);


} // end namepace {utility_pkg}
#endif
