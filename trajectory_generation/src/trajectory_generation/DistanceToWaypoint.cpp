/**
 * @brief Calculate Distance to current waypoint definitions
 * @file DistanceToWaypoint.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <trajectory_generation/DistanceToWaypoint.h>

using namespace trajectory_generation;

DistanceToWaypoint::DistanceToWaypoint(ros::NodeHandle& nh)
{
    std::string feedback_selected_topic = asl_gremlin_pkg::GetParam_with_shutdown<std::string>
                                            (nh, "state_feedback/feedback_selected", __LINE__);

    vehicle_state_ = new asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::VehicleState>
                                                        (nh, feedback_selected_topic);

    if (!nh.getParam("sim/waypoint_proximity",waypoint_proximity_))
    { ROS_WARN("couldn't access parameter /%s/sim/waypoint_proximity, default value of 0.8m is set",
                ros::this_node::getNamespace().c_str()); }

    ros::spinOnce();
}

DistanceToWaypoint::~DistanceToWaypoint()
{ delete vehicle_state_; }

void DistanceToWaypoint::set_waypoint(double x, double y)
{
    x_wp_ = x; y_wp_ = y;
    ++local_counter_;
}

bool DistanceToWaypoint::is_reached_waypoint()
{
    double x_current = (vehicle_state_->get_data())->pose.point.x;
    double y_current = (vehicle_state_->get_data())->pose.point.y;

    if( std::sqrt( std::pow(x_current - x_wp_,2) + 
                        (std::pow(y_current - y_wp_, 2))) < 
            waypoint_proximity_ )
    {
    	ROS_INFO("Reached waypoint \033[1;37m(x,y)\033[0;m:= (%f, %f)",x_wp_,y_wp_);
    	return true;
    }
    else
    { return false; }
}
void DistanceToWaypoint::reset_vehicle_pos()
{
    local_counter_ = 0;
    (vehicle_state_->get_data())->pose.point.x = 0.0;
    (vehicle_state_->get_data())->pose.point.y = 0.0;
}
