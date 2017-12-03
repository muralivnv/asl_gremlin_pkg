/**
 * @brief Generate CircularTrajectory header and definitions
 * @file CircularTrajectory.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _trajectory_generation_CIRCULARTRAJECTORY_H_
#define _trajectory_generation_CIRCULARTRAJECTORY_H_

#include <iostream>
#include <ros/ros.h>
#include <array>
#include <asl_gremlin_msgs/RefTraj.h>
#include <ros/time.h>

namespace trajectory_generation{


template<typename ParamType>
class CircularTrajectory{
	ParamType* params_;
    asl_gremlin_msgs::RefTraj* ref_traj_ptr_;
	
	int turn_dir_ = 0;
	double circle_start_angle_ = 0.0, circle_end_angle_ = 0.0;
    double X_center_ = 0.0, Y_center_ = 0.0;
    double min_turn_rad_ = 2.0 /*(m)*/, const_turn_vel_ = 0.5 /*(m/sec)*/;
    double t_initial_ = ros::Time::now(), final_time_ = 8.0; // (sec)

    public:
        CircularTrajectory(const ParamType*);
        ~CircularTrajectory(){
            delete params_;
        }
        void calc_params(double, double);
        void generate_traj();
        asl_gremlin_msgs::RefTraj* get_trajectory();
};

template<typename ParamType>
CircularTrajectory<ParamType>::CircularTrajectory(const ParamType* param)
{ params_ = param; }

template<typename ParamType>
CircularTrajectory<ParamType>::calc_params(double theta_current, double theta_req)
{
    theta_req = wrapTo2Pi(theta_req);
    theta_current = wrapTo2Pi(theta_current);

    double delta_theta_left = theta_req - theta_current;
    double delta_theta_right = wrapTo2Pi(theta_current - (theta_req - 2*M_PI));
    
    double start_horiz = 0;
    if (std::fabs(delta_theta_left) < std::fabs(delta_theta_right))
    {
        turn_dir_ = 1;
        start_horiz = 0;
    }
    else
    {
        turn_dir = -1;
        start_horiz = pi;
    }

    double perpendicular_to_theta_current = start_horiz + (theta_current - M_PI/2);
    double perpendicular_to_theta_req = wrapTo2Pi(theta_req - turn_dir_*M_PI/2);
    
    circle_end_angle_ = wrapTo2Pi(turn_dir*(perpendicular_to_theta_req - perpendicular_to_theta_current));
    X_center = X_



}





}
