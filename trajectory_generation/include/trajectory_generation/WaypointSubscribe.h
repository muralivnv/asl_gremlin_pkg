#ifndef _trajectory_generation_WAYPOINTSUBSCRIBE_H_
#define _trajectory_generation_WAYPOINTSUBSCRIBE_H_

#include <iostream> 
#include <vector> 
#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_generation/waypointSetConfig.h>
#include <dynamic_reconfigure/server.h>
#include <utility_pkg/str_manip.h>

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

        bool received_waypoints(){
            return received_wp_;
        }
};


#endif
