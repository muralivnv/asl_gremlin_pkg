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
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_pkg/GetParam.h>
#include <ros/time.h>
#include <utility_pkg/utilities.h>
#include "TrajectoryBase.h"

namespace trajectory_generation{

template<typename ParamType>
class CircularTrajectory : public TrajectoryBase{
    public:
        CircularTrajectory(ros::NodeHandle&, ParamType*);
        ~CircularTrajectory(){
            params_ = nullptr;
            delete ref_traj_ptr_;
            delete vehicle_state_;
        }
        void update_start_time(double) override;
        void set_ini_pose(double = 0.0, double = 0.0, double = 0.0) override;
        void set_final_pose(double, double, double = 0.0) override;
        void set_current_pose_as_ini() override;
        void calc_params() override;
        void generate_traj(double) override;
        asl_gremlin_msgs::RefTraj* get_trajectory() override;

    private:
        ParamType* params_ = nullptr;
        asl_gremlin_msgs::RefTraj* ref_traj_ptr_;
        asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::VehicleState>* vehicle_state_;
        
        int turn_dir_ = 0;
        double circle_start_angle_ = 0.0, circle_end_angle_ = 0.0;
        double circle_center_x_ = 0.0, circle_center_y_ = 0.0;
        double current_pose_x_ = 0.0, current_pose_y_ = 0.0;
        double current_heading_ = 0.0, required_heading_ = 0.0;
        
        double t_initial_ = ros::Time::now().toSec(), 
               final_time_ = 8.0; // (sec)

        int msg_count_ = 0;
};

template<typename ParamType>
CircularTrajectory<ParamType>::CircularTrajectory(ros::NodeHandle& nh, ParamType* param)
{
    ref_traj_ptr_ = new asl_gremlin_msgs::RefTraj();
	std::string feedback_selected_topic = asl_gremlin_pkg::GetParam_with_shutdown<std::string>
                                            (nh, "state_feedback/feedback_selected", __LINE__);

    vehicle_state_ = new asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::VehicleState>
                                                        (nh, feedback_selected_topic); 
    params_ = param; 
}

template<typename ParamType>
void CircularTrajectory<ParamType>::update_start_time(double time)
{ t_initial_ = time; }

template<typename ParamType>
void CircularTrajectory<ParamType>::set_ini_pose(   double x_ini,
                                                    double y_ini,
                                                    double theta_ini)
{
    current_pose_x_ = x_ini;
    current_pose_y_ = y_ini;
    current_heading_ = (vehicle_state_->get_data())->heading*M_PI/180.0;
}

template<typename ParamType>
void CircularTrajectory<ParamType>::set_final_pose( double x_final,
                                                    double y_final,
                                                    double theta_final)
{
    update_start_time(ros::Time::now().toSec());
    required_heading_ = std::atan2( y_final - current_pose_y_,
                                    x_final - current_pose_x_);
}

template<typename ParamType>
void CircularTrajectory<ParamType>::set_current_pose_as_ini()
{
    ros::spinOnce();
    current_pose_x_ = (vehicle_state_->get_data())->pose.point.x;
    current_pose_y_ = (vehicle_state_->get_data())->pose.point.y;
    current_heading_ = (vehicle_state_->get_data())->heading*M_PI/180.0;
}


template<typename ParamType>
void CircularTrajectory<ParamType>::calc_params()
{
    ros::spinOnce();
   
    required_heading_ = utility_pkg::wrapTo2Pi(required_heading_);
    current_heading_ = utility_pkg::wrapTo2Pi(current_heading_);

    double delta_theta_left = utility_pkg::wrapTo2Pi(required_heading_ - current_heading_);
    double delta_theta_right = utility_pkg::wrapTo2Pi(current_heading_ - (required_heading_ - 2*M_PI));

    double start_horiz = 0;
    if (std::fabs(delta_theta_left) < std::fabs(delta_theta_right))
    {
        turn_dir_ = 1;
        start_horiz = 0;
    }
    else
    {
        turn_dir_ = -1;
        start_horiz = M_PI;
    }

    double perpendicular_to_theta_current = start_horiz + (current_heading_ - M_PI/2);
    double perpendicular_to_theta_req = utility_pkg::wrapTo2Pi(required_heading_ - turn_dir_*M_PI/2);
    
    circle_end_angle_ = utility_pkg::wrapTo2Pi(turn_dir_*(perpendicular_to_theta_req - perpendicular_to_theta_current));
    circle_center_x_ = current_pose_x_ + params_->min_turn_rad*std::cos(M_PI - start_horiz + current_heading_ - M_PI/2);
    circle_center_y_ = current_pose_y_ + params_->min_turn_rad*std::sin(M_PI - start_horiz + current_heading_ - M_PI/2);

    final_time_ = params_->min_turn_rad*circle_end_angle_/(params_->const_turn_vel);
    
    if (turn_dir_ == -1)
    { circle_start_angle_ = perpendicular_to_theta_req; }
    else
    { circle_start_angle_ = perpendicular_to_theta_current; }
}


template<typename ParamType>
void CircularTrajectory<ParamType>::generate_traj(double time)
{
    double t_rel = (time - t_initial_);
    if (turn_dir_ == -1)
    {
        t_rel = final_time_ - t_rel;
        t_rel = std::max(0.0, t_rel);
    }
    else
    { t_rel = std::min(t_rel, final_time_); }

    ref_traj_ptr_->x = circle_center_x_ + params_->min_turn_rad * 
                                        std::cos(circle_end_angle_*t_rel/final_time_ + circle_start_angle_);
    
    ref_traj_ptr_->y = circle_center_y_ + params_->min_turn_rad * 
                                        std::sin(circle_end_angle_*t_rel/final_time_ + circle_start_angle_);
    
    ref_traj_ptr_->x_dot = -turn_dir_*params_->min_turn_rad*(circle_end_angle_/final_time_)*
                                        std::sin(circle_end_angle_*t_rel/final_time_ + circle_start_angle_);

    ref_traj_ptr_->y_dot = turn_dir_*params_->min_turn_rad*(circle_end_angle_/final_time_)*
                                        std::cos(circle_end_angle_*t_rel/final_time_ + circle_start_angle_);

    ref_traj_ptr_->x_ddot = -params_->min_turn_rad*std::pow((circle_end_angle_/final_time_),2)*
                                        std::cos(circle_end_angle_*t_rel/final_time_ + circle_start_angle_);

    ref_traj_ptr_->y_ddot = -params_->min_turn_rad*std::pow((circle_end_angle_/final_time_),2)*
                                        std::sin(circle_end_angle_*t_rel/final_time_ + circle_start_angle_);

    ref_traj_ptr_->theta = std::atan2(ref_traj_ptr_->y_dot, ref_traj_ptr_->x_dot);
    ref_traj_ptr_->theta_dot = 0.0;
    ref_traj_ptr_->theta_ddot = 0.0;

    ref_traj_ptr_->header.seq = msg_count_;
    ref_traj_ptr_->header.stamp = ros::Time::now();

    ++msg_count_;
}


template<typename ParamType>
inline asl_gremlin_msgs::RefTraj* 
CircularTrajectory<ParamType>::get_trajectory()
{ return ref_traj_ptr_; }

} // end namespace {trajectory_generation}

#endif
