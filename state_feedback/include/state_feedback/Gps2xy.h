/**
 * @brief Global to local co-ordinate conversion header
 * @file Gps2xy.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#ifndef _state_feedback_GPS2XY_H_
#define _state_feedback_GPS2XY_H_

#include <iostream>
#include <cmath>
#include <array>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>

namespace state_feedback{
class Gps2xy{
    double lat_ini = 0.0, lon_ini = 0.0, alt_ini = 0.0;
    double lat = 0.0, lon = 0.0, alt = 0.0;

    std::array<double, 3>   pos_ECEF{{0.0,0.0,0.0}}, 
                            pos_ECEF_ini{{0.0,0.0,0.0}}, 
                            pos_ENU{{0.0,0.0,0.0}};

    const double rad_earth = 6371000;  // (m)
    const double semi_major = 6378137;  // (m)
    const double semi_minor = 6356752.3142; // (m)
    const double eccen_sq = 0.0066943800; // eccentricity squared

public:
    void update_ecef_ini();
    void geod2ecef();
    void ecef2enu();
    double pos_ENU_x() const;
    double pos_ENU_y() const;
    double pos_ENU_z() const;

    // subscriber callback functions
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr&);
    void reset();
    void ini_cond_callback(const std_msgs::Float32MultiArray::ConstPtr&);
};

} //end namespace {state_feedback}
#endif
