/**
 * @brief WaypointSubscriber server header
 * @file  WaypointSubscribe.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _trajectory_generation_WAYPOINTSUBSCRIBE_H_
#define _trajectory_generation_WAYPOINTSUBSCRIBE_H_

#include <iostream> 
#include <vector> 
#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_generation/waypointSetConfig.h>
#include <dynamic_reconfigure/server.h>
#include <utility_pkg/str_manip.h>
#include <std_msgs/Bool.h>

class WaypointSubscribe{
        
        std::vector<double> x_waypoints_, y_waypoints_;
        int current_waypoint_ptr_ = 0;
        bool received_wp_ = false;

        dynamic_reconfigure::Server<trajectory_generation::waypointSetConfig> dr_wp_srv_;
        dynamic_reconfigure::Server<trajectory_generation::waypointSetConfig>::CallbackType fun_;

    public:
       
        WaypointSubscribe(ros::NodeHandle& );
        void dynamic_reconfigure_waypointSet_callback(trajectory_generation::waypointSetConfig&, uint32_t);
       
        std::vector<double> get_current_waypoint();
        std::vector<double> get_next_waypoint();
        void reset_counter();

        bool received_waypoints(){
            return received_wp_;
        }
};


#endif
