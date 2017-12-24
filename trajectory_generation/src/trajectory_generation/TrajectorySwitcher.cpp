/**
 * @brief Trajectory switcher definitions
 * @file TrajectorySwitcher.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <trajectory_generation/TrajectorySwitcher.h>

using namespace trajectory_generation;

double trajectory_generation::delta_theta(double theta_act, double theta_des)
{
	/* 			-- IMPORTANT NOTE -- 
	*	The output of this function will be multiplied by -ve sign 
	*	as there is a negative multiplying the error in heading
	* 	so as to obtain the correct direction of rotation
	*
	*		(wr - wl) = (b/r)(theta_dot_cmd - lambda_theta * delta_theta)
	*									  __^__
	*/

	// normal difference
	double delta1 = theta_act - theta_des;

	// angle difference in "Clockwise"
	double delta2 = -((M_PI - theta_act) - (-M_PI - theta_des));

	// angle difference in "Anti-Clockwise"
	double delta3 = ((M_PI - theta_des) - (-M_PI - theta_act));

	double delta = 0.0;

	// pick the smallest rotational direction
	if (std::fabs(delta1) < std::fabs(delta2))
	{	delta = delta1;	}
	else
	{   delta = delta2; }

	if (std::fabs(delta) > std::fabs(delta3))
	{   delta = delta3;	}
	
	return delta;
}

TrajectorySwitcher::TrajectorySwitcher(ros::NodeHandle& nh)
{
	std::string feedback_selected_topic;
    if(!nh.getParam("state_feedback/feedback_selected", feedback_selected_topic))
    { feedback_selected_topic = "state_feedback/selected_feedback"; }

    vehicle_state_ = new asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::VehicleState>
                                                        (nh, feedback_selected_topic);

    if (!nh.getParam("sim/waypoint_proximity",waypoint_proximity_))
    { ROS_WARN("couldn't access parameter /%s/sim/waypoint_proximity, default value of 0.8m is set",
                ros::this_node::getNamespace().c_str()); }

    if (!nh.getParam("sim/angle_align_tolerance",turn_tolerance_))
    { ROS_WARN("couldn't access parameter /%s/sim/angle_align_tolerance, default value of 25deg is set",
                ros::this_node::getNamespace().c_str()); }

    ros::spinOnce();
}

TrajectorySwitcher::~TrajectorySwitcher()
{ delete vehicle_state_; }

void TrajectorySwitcher::change_next_desired_state(double x, double y)
{
	do_console_output_ = true;
    x_wp_ = x; y_wp_ = y;

    ros::spinOnce();
    
    double x_current = (vehicle_state_->get_data())->pose.point.x;
    double y_current = (vehicle_state_->get_data())->pose.point.y;

    theta_req_ = std::atan2(y_wp_ - y_current, x_wp_ - x_current);
}

bool TrajectorySwitcher::need_to_switch_trajectory()
{
    double x_current = (vehicle_state_->get_data())->pose.point.x;
    double y_current = (vehicle_state_->get_data())->pose.point.y;

    double theta_current =  (vehicle_state_->get_data())->heading * M_PI/180.0;
    
    if (switch_condition_ == trajSwitchCond::dist_to_waypoint)
    {
        if( std::sqrt( std::pow(x_current - x_wp_,2) + 
                    (std::pow(y_current - y_wp_, 2))) < 
                waypoint_proximity_ )
        {
        	if (do_console_output_)
            { 
            	ROS_INFO("Reached waypoint \033[1;37m(x,y)\033[0;m:= (%f, %f)",x_wp_,y_wp_); 
            	do_console_output_ = false;
    		}
            return true;
        }
        else
        { return false; }
    }
    else if (switch_condition_ == trajSwitchCond::delta_theta_to_ref)
    {
        if ( std::fabs(delta_theta(theta_current, theta_req_)) <= turn_tolerance_)
        {
        	if(do_console_output_)
            { 
            	ROS_INFO("Aligned rover towards next waypoint\033[1;37m(x,y)\033[0;m:= (%f,%f)",x_wp_,y_wp_); 
            	do_console_output_ = false;
    		}
            return true;
        }
        else
        {return false;}
    }

}

bool TrajectorySwitcher::current_hdg_within_tolerance_to_ref()
{
    ros::spinOnce();

    double current_heading = (vehicle_state_->get_data())->heading * M_PI/180.0;
    return std::fabs(delta_theta(current_heading, theta_req_)) <= turn_tolerance_;
}

void TrajectorySwitcher::change_switch_condition(trajSwitchCond condition)
{ 
	switch_condition_ = condition; 
	do_console_output_ = true;
}

void TrajectorySwitcher::reset_vehicle_state()
{
	do_console_output_ = true;
    (vehicle_state_->get_data())->pose.point.x = 0.0;
    (vehicle_state_->get_data())->pose.point.y = 0.0;
}
