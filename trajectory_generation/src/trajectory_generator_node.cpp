#include <trajectory_generation/MinimumJerkTrajectory.h>
#include <trajectory_generation/trajectory_publisher.h>
#include <trajectory_generation/WaypointSubscribe.h>
#include <trajectory_generation/DistanceToWaypoint.h>
#include <std_msgs/Bool.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <asl_gremlin_pkg/GetParam.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <string>

using namespace trajectory_generation;

struct traj_params{
   double accel_max = 0.5;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv , "trajectory_generation"); 
    ros::NodeHandle traj_nh;

    traj_params params;

    TrajectoryBase* min_jerk_traj = new MinimumJerkTrajectory<traj_params>(&params);
    WaypointSubscribe waypoint_stack(traj_nh);
    DistanceToWaypoint* dist_to_wp = new DistanceToWaypoint(traj_nh);

    asl_gremlin_pkg::SubscribeTopic<std_msgs::Bool> sim(traj_nh, ros::this_node::getNamespace()+"/start_sim"); 
    
    std::string traj_pub_name;
    
    traj_pub_name = asl_gremlin_pkg::GetParam_with_shutdown<std::string>(traj_nh, "/trajectory/publisher_topic", __LINE__);

    ros::Publisher traj_pub = traj_nh.advertise<asl_gremlin_msgs::RefTraj>(traj_pub_name, 10);

    int rate = 10;
    if (!traj_nh.getParam(ros::this_node::getNamespace()+"/sim/rate", rate))
    {
        ROS_WARN("Unable access parameter $robot_name/sim/rate, setting rate as 10Hz");
    }
    ros::Rate loop_rate(rate);

    bool updated_ini_params = false;
    std::vector<double> waypoint(2,0);
    
    ROS_INFO("Initialized /trajectory_generator");
    while(ros::ok())
    {
        if ( (sim.get_data())->data )
        {
            if (!waypoint_stack.received_waypoints())
            { 
                ROS_ERROR("waypoint stack is empty: USE COMMAND\n" 
                            "\t\t\t\t rosrun trajectory_generation waypointSet_client -x \"x1,x2...\""
                            " -y \"y1,y2,...\" \n"
                            "\t\t\t in new terminal");
            }
            else
            {
                if ( updated_ini_params == false )
                {
                    ROS_INFO_ONCE("Started creating trajectory for given waypoints");
                    min_jerk_traj->set_ini_pose(0.0, 0.0);

                    waypoint = waypoint_stack.get_current_waypoint();
                    dist_to_wp->set_waypoint(waypoint[0], waypoint[1]);

                    min_jerk_traj->set_final_pose(waypoint[0], waypoint[1]);
                    min_jerk_traj->calc_coeff();

                    updated_ini_params = true;
                }

                if ( dist_to_wp->is_reached_waypoint() )
                {
                    min_jerk_traj->set_current_traj_value_to_ini();

                    waypoint = waypoint_stack.get_next_waypoint();
                    dist_to_wp->set_waypoint(waypoint[0], waypoint[1]);

                    min_jerk_traj->set_final_pose(waypoint[0], waypoint[1]);
                    min_jerk_traj->calc_coeff();
                }

                min_jerk_traj->generate_traj(ros::Time::now().toSec());
            }
        }

        trajectory_generation::publish_trajectory(traj_pub, min_jerk_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
