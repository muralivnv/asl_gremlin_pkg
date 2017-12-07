/**
 * @brief track_other_rover
 * @file track_other_rover.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <trajectory_generation/MinimumJerkTrajectory.h>
#include <trajectory_generation/CircularTrajectory.h>
#include <trajectory_generation/TrajectorySwitcher.h>
#include <std_msgs/Bool.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <asl_gremlin_pkg/GetParam.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <string>
#include <chrono>
#include <thread>
#include <memory>

using namespace trajectory_generation;

struct traj_params{
   double accel_max = 0.1;
    traj_params(ros::NodeHandle& nh){
        if (!nh.getParam("sim/max_accel",accel_max))
        { 
            ROS_WARN("Unable to access param '%s/sim/max_accel', setting to 0.1m/sec^2",
                    ros::this_node::getNamespace().c_str());
            accel_max = 0.1;
        }
    }
};

struct circle_params{
    double min_turn_rad = 2; // (m)
    double const_turn_vel = 0.5; // (m/sec)
    circle_params(ros::NodeHandle& nh){
        if (!nh.getParam("sim/min_turn_radius",min_turn_rad))
        {
            ROS_WARN("Unable to access param '%s/sim/min_turn_radius', setting to 2m",
                    ros::this_node::getNamespace().c_str());
            min_turn_rad = 2.0;
        }
        if (!nh.getParam("sim/const_turn_vel",const_turn_vel))
        {
            ROS_WARN("Unable to access param '/%s/sim/const_turn_vel', setting to 0.5m/sec",
                    ros::this_node::getNamespace().c_str());
            const_turn_vel = 0.5;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv , "track_other_rover"); 

    ros::NodeHandle traj_nh;

    traj_params params(traj_nh);
    circle_params params_circle(traj_nh);

    TrajectoryBase* traj_gen = nullptr;
    MinimumJerkTrajectory<traj_params>* min_jerk_traj = 
                                        new MinimumJerkTrajectory<traj_params>(traj_nh, &params);
    
    CircularTrajectory<circle_params>* circular_traj = 
                                        new CircularTrajectory<circle_params>(traj_nh, &params_circle);
    traj_gen = min_jerk_traj;

    std::unique_ptr<TrajectorySwitcher> switch_trajectory = 
                            std::make_unique<TrajectorySwitcher>(traj_nh);

    asl_gremlin_pkg::SubscribeTopic<std_msgs::Bool> sim(traj_nh,"start_sim"); 
    
    std::string traj_pub_name;
    
    traj_pub_name = asl_gremlin_pkg::GetParam_with_shutdown<std::string>
                    (traj_nh, "trajectory/publisher_topic", __LINE__);

    ros::Publisher traj_pub = traj_nh.advertise<asl_gremlin_msgs::RefTraj>(traj_pub_name, 10);

    double rate = 10.0;
    if (!traj_nh.getParam("sim/rate", rate))
    {
        ROS_WARN("Unable to access parameter /%s/sim/rate, setting rate as 10Hz",
                    ros::this_node::getNamespace().c_str());
    }
    ros::Rate loop_rate(rate);

    bool aligned_rover = false;
    std::vector<double> waypoint(2,0);
    
    ROS_INFO("\033[1;32mInitialized\033[0;m:= %s",ros::this_node::getName().c_str());
    while(ros::ok())
    {
        if ( (sim.get_data())->data )
        {
            for_every(30sec);
            waypoint[0] = (asl_gremlin1_state->get_data())->pose.point.x;
            waypoint[1] = (asl_gremlin1_state->get_data())->pose.point.y;

            switch_trajectory->change_next_desired_state(waypoint[0], waypoint[1]);


            if (!switch_trajectory->current_hdg_within_tolerance_to_ref() && !aligned_rover)
            {
                traj_gen = circular_traj;
                switch_trajectory->change_switch_condition(trajSwitchCond::delta_theta_to_ref);
                waypoint_stack.decrement_counter();
                aligned_rover = true;
            }
            else
            {
                traj_gen = min_jerk_traj;
                switch_trajectory->change_switch_condition(trajSwitchCond::dist_to_waypoint);
                aligned_rover = false;
            }

            traj_gen->set_current_pose_as_ini();
            traj_gen->set_final_pose(waypoint[0], waypoint[1]);
            traj_gen->calc_params();
            traj_gen->generate_traj(ros::Time::now().toSec());
        }
        else if (!(sim.get_data())->data && updated_ini_params)
        {
            ROS_INFO("\033[1;31mStopped\033[0;m:= Generating trajectory for given waypoints");
            switch_trajectory->reset_vehicle_state();
            waypoint_stack.reset_counter();
            aligned_rover = false;
        }

        if (traj_gen != nullptr)
        { traj_pub.publish(*(traj_gen->get_trajectory())); }
        else
        { traj_pub.publish(*(min_jerk_traj->get_trajectory())); }

        ros::spinOnce();
        loop_rate.sleep();
    }

    traj_gen = nullptr;
    delete min_jerk_traj;
    delete circular_traj;

    return EXIT_SUCCESS;
}
