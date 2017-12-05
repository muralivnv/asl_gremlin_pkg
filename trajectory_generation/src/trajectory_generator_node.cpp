/**
 * @brief trajectory_generator_node
 * @file trajectory_generator_node.cpp
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
#include <trajectory_generation/WaypointSubscribe.h>
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

int main(int argc, char** argv)
{
    ros::init(argc, argv , "trajectory_generation"); 

    ros::NodeHandle traj_nh;

    traj_params params(traj_nh);

    std::unique_ptr<TrajectoryBase> min_jerk_traj = 
                            std::make_unique<MinimumJerkTrajectory<traj_params>>(traj_nh, &params);
    WaypointSubscribe waypoint_stack(traj_nh);

    std::unique_ptr<TrajectorySwitcher> dist_to_wp = 
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

    bool updated_ini_params = false;
    std::vector<double> waypoint(2,0);
    
    ROS_INFO("\033[1;32mInitialized\033[0;m:= %s",ros::this_node::getName().c_str());
    while(ros::ok())
    {
        if ( (sim.get_data())->data )
        {
            if (!waypoint_stack.received_waypoints())
            { ROS_ERROR("Mismatch waypoint size for (X,Y) or Waypoints are not specified"); }
            else
            {
                if ( !updated_ini_params )
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    waypoint_stack.reset_counter();
                    dist_to_wp->reset_vehicle_state();
                    ROS_INFO("\033[1;32mInitialized\033[0;m:= Generating trajectory for given waypoints");
                    min_jerk_traj->set_ini_pose(0.0, 0.0);

                    waypoint = waypoint_stack.get_current_waypoint();
                    dist_to_wp->change_next_desired_state(waypoint[0], waypoint[1]);
                    dist_to_wp->change_switch_condition(trajSwitchCond::dist_to_waypoint);

                    min_jerk_traj->set_final_pose(waypoint[0], waypoint[1]);
                    min_jerk_traj->calc_params();

                    updated_ini_params = true;
                }

                if ( dist_to_wp->need_to_switch_trajectory() )
                {
                    min_jerk_traj->set_current_pose_as_ini();

                    waypoint = waypoint_stack.get_next_waypoint();
                    
                    if (waypoint.size() == 1)
                    { 
                        ros::spinOnce(); 
                        dist_to_wp->reset_vehicle_state();
                        continue; 
                    }
                    
                    dist_to_wp->change_next_desired_state(waypoint[0], waypoint[1]);

                    min_jerk_traj->set_final_pose(waypoint[0], waypoint[1]);
                    min_jerk_traj->calc_params();
                }
                min_jerk_traj->generate_traj(ros::Time::now().toSec());
            }
        }
        else if (!(sim.get_data())->data && updated_ini_params)
        {
            ROS_INFO("\033[1;31mStopped\033[0;m:= Generating trajectory for given waypoints");
            updated_ini_params = false;
            dist_to_wp->reset_vehicle_state();
            waypoint_stack.reset_counter();
        }

        traj_pub.publish(*(min_jerk_traj->get_trajectory()));

        ros::spinOnce();
        loop_rate.sleep();
    }
}
