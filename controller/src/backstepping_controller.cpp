#include <controller/BackStepping.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int16MultiArray.h>


int main(int argc, char** argv)
{

    ros::init(argc, argv , "backstepping_controller"); 
    ros::NodeHandle ctrl_nh;

    dynamic_reconfigure::Server<trajectory_generation::waypointSetConfig> dr_gain_srv;
    dynamic_reconfigure::Server<trajectory_generation::waypointSetConfig>::CallbackType fun;
    fun = boost::bind(&TrajGen::dynamic_reconfigure_waypoint_callback, &traj_gen, _1, _2);
    dr_wp_srv.setCallback(fun);


    ros::Rate loop_rate(10);
