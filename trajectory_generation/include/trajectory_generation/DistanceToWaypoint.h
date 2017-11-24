/**
 * @brief Calculate distance to current waypoint header
 * @file DistanceToWaypoint.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#ifndef _trajectory_generation_DISTANCETOWAYPOINT_H_
#define _trajectory_generation_DISTANCETOWAYPOINT_H_

#include <ros/ros.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_pkg/GetParam.h>
#include <cmath>
#include <string>

namespace trajectory_generation{

class DistanceToWaypoint{
    double x_wp_, y_wp_;
    double waypoint_proximity_;
    int local_counter_ = 0;

    asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::VehicleState>* vehicle_state_;

    public:
        DistanceToWaypoint(ros::NodeHandle&);
        ~DistanceToWaypoint();
        
        void set_waypoint(double, double);
        bool is_reached_waypoint();
        void reset_vehicle_pos();
};

} // end namespace{trajectory_generation}
#endif
