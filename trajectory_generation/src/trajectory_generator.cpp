#include <trajectory_generation/TrajGen.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include <trajectory_generation/ref_traj.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv , "trajectory_generation"); 
    ros::NodeHandle traj_nh;

    TrajGen traj_gen(ros::Time::now().toSec());

    ros::Publisher traj_pub = traj_nh.advertise<trajectory_generation::ref_traj>("/asl_gremlin1/trajectory/ref_traj", 10);

    ros::Subscriber start_sim_sub = traj_nh.subscribe("/asl_gremlin1/start_sim",10,
                                                        &TrajGen::start_sim_callback, &traj_gen);

    dynamic_reconfigure::Server<trajectory_generation::waypointSetConfig> dr_wp_srv;
    dynamic_reconfigure::Server<trajectory_generation::waypointSetConfig>::CallbackType fun;
    fun = boost::bind(&TrajGen::dynamic_reconfigure_waypoint_callback, &traj_gen, _1, _2);
    dr_wp_srv.setCallback(fun);


    ros::Rate loop_rate(10);

    int wp_trig = 0;
    bool updated_ini_params = false;

    traj_gen.set_publisher(traj_pub);


    while(ros::ok())
    {
        if ( traj_gen.is_start_sim() )
        {
            if ( updated_ini_params == false )
            {
                traj_gen.set_ini_cond(0.0, 0.0);
                traj_gen.select_wp(wp_trig);              // zero-indexing
                traj_gen.set_start_time();
                traj_gen.calc_coeff();
                updated_ini_params = true;
            }

            if ( traj_gen.is_reached_waypoint() )
            {
                ++wp_trig;
                traj_gen.set_current_to_ini();
                traj_gen.select_wp(wp_trig);
                traj_gen.set_start_time();
                traj_gen.calc_coeff();
            }

            traj_gen.gen_traj(ros::Time::now().toSec());
        }

        traj_gen.publish();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
