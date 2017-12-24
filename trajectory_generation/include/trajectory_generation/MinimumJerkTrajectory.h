/**
 * @brief Generate MinimumJerkTrajectory header and definitions
 * @file MinimumJerkTrajectory.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _trajectory_generation_MINIMUMJERKTRAJECTORY_H_
#define _trajectory_generation_MINIMUMJERKTRAJECTORY_H_

#include <iostream>
#include "TrajectoryBase.h"
#include <array>
#include <asl_gremlin_msgs/RefTraj.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <ros/ros.h>
#include <ros/time.h>
#include "EvaluatePolynomial.h"

template<typename ParamType>
class MinimumJerkTrajectory : 
                    public TrajectoryBase {
    public:
        MinimumJerkTrajectory(ros::NodeHandle&, ParamType*);
        ~MinimumJerkTrajectory();
        void update_start_time(double) override;
        void set_ini_pose(double = 0.0,double = 0.0,double = 0.0) override;
        void set_final_pose(double,double,double = 0.0) override;
        void set_current_pose_as_ini() override;
        
        void calc_params() override;
        void generate_traj(double) override;

        asl_gremlin_msgs::RefTraj* get_trajectory() override;

    private:
        ParamType* params_ = nullptr;

        std::array<double, 6> x_coeff_{{0,0,0,0,0,0}};
        std::array<double, 6> y_coeff_{{0,0,0,0,0,0}};

        double x_ini_ = 0.0, y_ini_ = 0.0;
        double x_final_ = 0.0, y_final_ = 0.0;
        double t_initial_ = 0.0, t_final_ = 0.0;
        
        asl_gremlin_msgs::RefTraj ref_traj_obj_;
        void calc_x_coeff_(double);
        void calc_y_coeff_(double);
        int msg_count = 0;
        asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::VehicleState>* vehicle_state_;
};

template<typename ParamType>
MinimumJerkTrajectory<ParamType>::MinimumJerkTrajectory(ros::NodeHandle& nh, ParamType* params)
{
	std::string feedback_selected_topic;
    if(!nh.getParam("state_feedback/feedback_selected", feedback_selected_topic))
    { feedback_selected_topic = "state_feedback/selected_feedback"; }

    vehicle_state_ = new asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::VehicleState>
                                                        (nh, feedback_selected_topic); 
	params_ = params;
	ros::spinOnce();
}

template<typename ParamType>
MinimumJerkTrajectory<ParamType>::~MinimumJerkTrajectory()
{ 
	delete vehicle_state_;
    params_ = nullptr;
}

template<typename ParamType>
void MinimumJerkTrajectory<ParamType>::update_start_time(double t_initial)
{ t_initial_ = t_initial; }

template<typename ParamType>
void MinimumJerkTrajectory<ParamType>::set_ini_pose(    double x_ini,
                                                        double y_ini, 
                                                        double theta_ini)
{
    x_ini_ = x_ini;
    y_ini_ = y_ini;
}

template<typename ParamType>
void MinimumJerkTrajectory<ParamType>::set_final_pose(  double x_final, 
                                                        double y_final,
                                                        double theta_final)
{
    update_start_time(ros::Time::now().toSec());
    x_final_ = x_final;
    y_final_ = y_final;
}

template<typename ParamType>
void MinimumJerkTrajectory<ParamType>::set_current_pose_as_ini()
{
    x_ini_ = (vehicle_state_->get_data())->pose.point.x;
    y_ini_ = (vehicle_state_->get_data())->pose.point.y;
}

template<typename ParamType> 
void MinimumJerkTrajectory<ParamType>::calc_params()
{
    // calculate final time required
   double delta_x = fabs(x_final_ - x_ini_);
   double delta_y = fabs(y_final_ - y_ini_);

   double t_final_x = sqrt(10/(sqrt(3)) * delta_x/params_->accel_max);
   double t_final_y = sqrt(10/(sqrt(3)) * delta_y/params_->accel_max);
    
   t_final_ = std::max(t_final_x, t_final_y) + std::max(delta_x, delta_y);

   calc_x_coeff_(t_final_);
   calc_y_coeff_(t_final_);
}

template<typename ParamType>
void MinimumJerkTrajectory<ParamType>::generate_traj(double time)
{
    double t_rel = (time - t_initial_);

    t_rel = std::min(t_rel, t_final_);

    ref_traj_obj_.x         = eval_poly< 5 >(t_rel, x_coeff_, orderOfDiff::position);
    ref_traj_obj_.x_dot     = eval_poly< 5 >(t_rel, x_coeff_, orderOfDiff::velocity);
    ref_traj_obj_.x_ddot    = eval_poly< 5 >(t_rel, x_coeff_, orderOfDiff::acceleration);

    ref_traj_obj_.y         = eval_poly< 5 >(t_rel, y_coeff_, orderOfDiff::position);
    ref_traj_obj_.y_dot     = eval_poly< 5 >(t_rel, y_coeff_, orderOfDiff::velocity);
    ref_traj_obj_.y_ddot    = eval_poly< 5 >(t_rel, y_coeff_, orderOfDiff::acceleration);

    ref_traj_obj_.theta      =   std::atan2(ref_traj_obj_.y_dot, ref_traj_obj_.x_dot);
    ref_traj_obj_.theta_dot  =   0;
    ref_traj_obj_.theta_ddot =   0;

    ref_traj_obj_.header.seq = msg_count;
    ref_traj_obj_.header.stamp = ros::Time::now();

    msg_count++;
}

template<typename ParamType>
asl_gremlin_msgs::RefTraj* MinimumJerkTrajectory<ParamType>::get_trajectory()
{
    return &ref_traj_obj_;
}

template<typename ParamType>
void MinimumJerkTrajectory<ParamType>::calc_x_coeff_(double t_final)
{
    x_coeff_[0] = x_ini_;
    x_coeff_[3] = 10*(x_final_ - x_ini_)/(std::pow(t_final,3)); 
    x_coeff_[4] = -15*(x_final_ - x_ini_)/(std::pow(t_final,4));
    x_coeff_[5] = 6*(x_final_ - x_ini_)/(std::pow(t_final,5));
}

template<typename ParamType>
void MinimumJerkTrajectory<ParamType>::calc_y_coeff_(double t_final)
{
    y_coeff_[0] = y_ini_;
    y_coeff_[3] = 10*(y_final_ - y_ini_)/(std::pow(t_final,3)); 
    y_coeff_[4] = -15*(y_final_ - y_ini_)/(std::pow(t_final,4));
    y_coeff_[5] = 6*(y_final_ - y_ini_)/(std::pow(t_final,5));
}

#endif
