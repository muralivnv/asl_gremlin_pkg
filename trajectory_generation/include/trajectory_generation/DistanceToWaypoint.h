#ifndef _trajectory_generation_DISTANCETOWAYPOINT_H_
#define _trajectory_generation_DISTANCETOWAYPOINT_H_

#include <ros/ros.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <utility_pkg/error_util.h>
#include <cmath>
#include <string>

namespace trajectory_generation{

class DistanceToWaypoint{
    double x_wp_, y_wp_;

    asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::VehicleState>* vehicle_state_;

    public:
        DistanceToWaypoint(ros::NodeHandle&);
        ~DistanceToWaypoint();
        
        void set_waypoint(double, double);
        bool is_reached_waypoint();
};

} // end namespace{trajectory_generation}
#endif
