/**
 * @brief WaypointSubscribe definitions
 * @file WaypointSubscribe.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <trajectory_generation/WaypointSubscribe.h>

WaypointSubscribe::WaypointSubscribe(ros::NodeHandle& nh)
{
    fun_ = boost::bind(&WaypointSubscribe::dynamic_reconfigure_waypointSet_callback,this,
                       _1, _2);

    dr_wp_srv_.setCallback(fun_);
    x_waypoints_ = {0};
    y_waypoints_ = {0};
}

void WaypointSubscribe::dynamic_reconfigure_waypointSet_callback(trajectory_generation::waypointSetConfig & config,  uint32_t level)
{
    received_wp_ = false;

    auto x_wp_tmp = utility_pkg::string_to_vector<double>(config.X_waypoint);
    auto y_wp_tmp = utility_pkg::string_to_vector<double>(config.Y_waypoint);

    if (config.Reset_Xwp)
    { x_waypoints_ = x_wp_tmp; }

    if (config.Concatenate_Xwp)
    {  std::copy(x_wp_tmp.begin(), x_wp_tmp.end(), std::back_inserter(x_waypoints_)); }

	if (config.Remove_Xwp)
	{ 
		std::for_each(std::begin(x_wp_tmp), std::end(x_wp_tmp),
						[&](auto n){x_waypoints_.erase(std::find(std::begin(x_waypoints_), std::end(x_waypoints_), n));});
	}

    if (config.Reset_Ywp)
    { y_waypoints_ = y_wp_tmp; }

    if (config.Concatenate_Ywp)
    {  std::copy(y_wp_tmp.begin(), y_wp_tmp.end(), std::back_inserter(y_waypoints_)); }

	if (config.Remove_Ywp)
	{ 
		std::for_each(std::begin(y_wp_tmp), std::end(y_wp_tmp),
                        [&](auto n){y_waypoints_.erase(std::find(std::begin(y_waypoints_), std::end(y_waypoints_), n));});
	}

    if (  x_waypoints_.size() == y_waypoints_.size() )
    { received_wp_ = true; }
    else
    { ROS_WARN("Unequal waypoint stack sizes between X and Y, rectify this before 'Rover Initialization'"); }
    
    ROS_INFO("\033[0;33mUpdated\033[0;m:= {Waypoints}--> \033[1;37mbeep boop beep\033[0;m");
       
    if (std::fabs(x_waypoints_[0]) < 2 && std::fabs(y_waypoints_[0]) < 2)
    {
        x_waypoints_.erase(std::begin(x_waypoints_));
        y_waypoints_.erase(std::begin(y_waypoints_));
    }

    std::cout << "\033[1;37mX_wp\033[0;m:= ";
    utility_pkg::print_stl_container(x_waypoints_);

    std::cout << "\n\033[1;37mY_wp\033[0;m:= ";
    utility_pkg::print_stl_container(y_waypoints_);
}

std::vector<double> WaypointSubscribe::get_current_waypoint()
{
    return {x_waypoints_[current_waypoint_ptr_],
                y_waypoints_[current_waypoint_ptr_] };
}

std::vector<double> WaypointSubscribe::get_next_waypoint()
{
    ++current_waypoint_ptr_;
    if (current_waypoint_ptr_ == x_waypoints_.size())
    {
        --current_waypoint_ptr_;
        return {0};
    }
    
    return {x_waypoints_[current_waypoint_ptr_],
                y_waypoints_[current_waypoint_ptr_] };
}

void WaypointSubscribe::reset_counter()
{ current_waypoint_ptr_ = 0; }
