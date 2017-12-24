/**
 * @brief Trajectory switching header
 * @file TrajectorySwitcher.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#ifndef _trajectory_generation_TRAJECTORYSWITCHER_H_
#define _trajectory_generation_TRAJECTORYSWITCHER_H_

#include <ros/ros.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <cmath>
#include <string>

namespace trajectory_generation{

enum trajSwitchCond{
    dist_to_waypoint,
    delta_theta_to_ref
};

double delta_theta(double, double);

class TrajectorySwitcher{
    double x_wp_, y_wp_, theta_req_;
    double waypoint_proximity_ = 0.8; // (m)
    double turn_tolerance_ = 25*M_PI/180.0; // (rad)

    int switch_condition_ = trajSwitchCond::dist_to_waypoint;
    bool do_console_output_ = true;

    asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::VehicleState>* vehicle_state_;

    public:
        TrajectorySwitcher(ros::NodeHandle&);
        ~TrajectorySwitcher();
        
        void change_next_desired_state(double, double);
        bool need_to_switch_trajectory();
        bool current_hdg_within_tolerance_to_ref();
        void change_switch_condition(trajSwitchCond);
        void reset_vehicle_state();
};

} // end namespace{trajectory_generation}
#endif
