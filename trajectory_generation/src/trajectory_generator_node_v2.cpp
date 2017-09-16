#include <trajectory_generation/MinimumJerkTrajectory.h>
#include <trajectory_generation/trajectory_publisher.h>
#include <trajectory_generation/WaypointSubscribe.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_generation/ref_traj.h>
#include <utility_pkg/str_manip.h>
#include <string>

struct traj_params{
   double accel_max = 0.5;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv , "trajectory_generation"); 
    ros::NodeHandle traj_nh;

    traj_params params;

    TrajectoryBase* min_jerk_traj = new MinimumJerkTrajectory<traj_params>(&params);
    WaypointSubscribe waypoint_stack;

    
    dynamic_reconfigure::Server<trajectory_generation::waypointSetConfig> dr_wp_srv;
    dynamic_reconfigure::Server<trajectory_generation::waypointSetConfig>::CallbackType fun;
    fun = boost::bind(&WaypointSubscribe::dynamic_reconfigure_waypointSet_callback, _1, _2);
    dr_wp_srv.setCallback(fun);

    std::string traj_pub_name;
    
    if (!traj_nh.getParam("/asl_gremlin/trajectory/publisher_topic", traj_pub_name))
    {
        ROS_ERROR("Can't acces parameter '/asl_gremlin/trajectory/publisher_topic ");
        ros::shutdown();
    }

    ros::Publisher traj_pub = traj_nh.advertise<trajectory_generation::ref_traj>(traj_pub_name, 10);

    ros::Rate loop_rate(10);

    int wp_trig = 0;
    bool updated_ini_params = false;
    std::vector<double> waypoint(2,0);
    
    while(ros::ok())
    {
        if ( updated_ini_params == false )
        {
            if ( updated_ini_params == false )
            {
                min_jerk_traj->set_ini_pose(0.0, 0.0);
                waypoint = waypoint_stack.get_current_waypoint();
                utility_pkg::print_stl_container(waypoint);
                min_jerk_traj->update_start_time(ros::Time::now().toSec());
                min_jerk_traj->set_final_pose(waypoint[0], waypoint[1]);
                min_jerk_traj->calc_coeff();
                updated_ini_params = true;
            }

            if ( false )
            {
                min_jerk_traj->set_current_traj_value_to_ini();
                waypoint = waypoint_stack.get_next_waypoint();
                min_jerk_traj->update_start_time(ros::Time::now().toSec());
                min_jerk_traj->set_final_pose(waypoint[0], waypoint[1]);
                min_jerk_traj->calc_coeff();
            }

            min_jerk_traj->generate_traj(ros::Time::now().toSec());
        }

        trajectory_generation::publish_trajectory(traj_pub, min_jerk_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
