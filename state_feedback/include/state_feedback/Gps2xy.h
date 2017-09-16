#ifndef STATE_FEEDBACK__GPS2XY_HPP
#define STATE_FEEDBACK__GPS2XY_HPP

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

    std::array<double, 3> pos_ECEF, pos_ECEF_ini, pos_ENU;

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
    void init_callback(const std_msgs::Bool::ConstPtr&);
    void ini_cond_callback(const std_msgs::Float32MultiArray::ConstPtr&);
};

} //end namespace {state_feedback}
#endif
