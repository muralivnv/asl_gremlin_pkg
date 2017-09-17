#ifndef _trajectory_generation_DISTANCETOWAYPOINT_H_
#define _trajectory_generation_DISTANCETOWAYPOINT_H_

#include <ros/ros.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <utility_pkg/error_util.h>
#include <cmath>
#include <string>

namespace trajectory_generation{

class DistanceToWaypoint{
    double x_wp_, y_wp_;
    double x_current_, y_current_;
    
    ros::Subscriber vehicle_state_sub_;

    public:
        DistanceToWaypoint(ros::NodeHandle&);
        
        void vehicle_state_callback(const asl_gremlin_msgs::VehicleState::ConstPtr );
        void set_waypoint(double, double);
        bool is_reached_waypoint();
};

} // end namespace{trajectory_generation}
#endif
