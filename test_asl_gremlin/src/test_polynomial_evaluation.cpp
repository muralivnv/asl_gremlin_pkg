#include <trajectory_generation/MinimumJerkTrajectory.h>
#include <trajectory_generation/trajectory_publisher.h>
#include <std_msgs/Bool.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_msgs/RefTraj.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <string>
#include <memory>
//#include <fstream>
//#include <cassert>

using namespace trajectory_generation;

struct traj_params{
   double accel_max = 0.5;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv , "test_trajectory_generation"); 
    ros::NodeHandle traj_nh;

    traj_params params;

    std::unique_ptr<TrajectoryBase> min_jerk_traj = std::make_unique<MinimumJerkTrajectory<traj_params>>(traj_nh, &params);
    asl_gremlin_pkg::SubscribeTopic<std_msgs::Bool> sim(traj_nh,"/asl_gremlin/start_sim"); 

    std::string traj_pub_name = "/asl_gremlin/trajectory_generation/test_poly";
    
    ros::Publisher traj_pub = traj_nh.advertise<asl_gremlin_msgs::RefTraj>(traj_pub_name, 10);
    ros::Rate loop_rate(10);

    bool updated_ini_params = false;
    std::vector<double> waypoint{10,20};
    
  //  std::ofstream file_write("reference_traj.dat");
  //  assert( file_write.is_open() );

  //  file_write << "#time  " <<"#x  "<<"#x_dot  "<<"#x_ddot  "<<"#y  "<<"#y_dot  "<<"y_ddot" <<'\n';
    
    double initial_time = 0;
    asl_gremlin_msgs::RefTraj* traj;

    while(ros::ok())
    {
        if ( (sim.get_data())->data )
        {
            if ( updated_ini_params == false )
            {
                min_jerk_traj->set_ini_pose(0.0, 0.0);

                min_jerk_traj->update_start_time(ros::Time::now().toSec());
                min_jerk_traj->set_final_pose(waypoint[0], waypoint[1]);
                min_jerk_traj->calc_coeff();

                updated_ini_params = true;
                initial_time = ros::Time::now().toSec();
            }

            min_jerk_traj->generate_traj(ros::Time::now().toSec());
            traj = min_jerk_traj->get_trajectory();
    //        file_write << (traj->header.stamp.sec + traj->header.stamp.nsec*1e-9) - initial_time;
    //        file_write << "  " << traj->x << "  "<< traj->x_dot << "  " << traj->x_ddot;
    //        file_write << "  " << traj->y << "  "<< traj->y_dot << "  " << traj->y_ddot;
    //        file_write << '\n';
        }

        trajectory_generation::publish_trajectory(traj_pub, min_jerk_traj);
        std::cout<<ros::this_node::getNamespace()<<'\n';

        ros::spinOnce();
        loop_rate.sleep();
    }

   // file_write.close();
}
