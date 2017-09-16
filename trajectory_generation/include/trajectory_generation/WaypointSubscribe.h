#ifndef TRAJECTORY_GENERATION__WAYPOINT_SUBSCRIBE_H
#define TRAJECTORY_GENERATION__WAYPOINT_SUBSCRIBE_H

#include <iostream> 
#include <vector> 
#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_generation/waypointSetConfig.h>
#include <dynamic_reconfigure/server.h>
#include <utility_pkg/str_manip.h>

class WaypointSubscribe{
        static std::vector<double> x_waypoints_, y_waypoints_;
        int current_waypoint_ptr_ = 0;
    public:
       static void dynamic_reconfigure_waypointSet_callback(trajectory_generation::waypointSetConfig&, uint32_t);
       
       std::vector<double> get_current_waypoint();
       std::vector<double> get_next_waypoint();
};


#endif
