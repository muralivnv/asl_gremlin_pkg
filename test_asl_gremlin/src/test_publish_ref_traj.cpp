#include <ros/ros.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <fstream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_publish_ref_traj");
    ros::NodeHandle nh;

    ros::Publisher ref_traj_pub = nh.advertise<asl_gremlin_msgs::RefTraj>
                            ("/asl_gremlin1/trajectory_generation/reference_trajectory",
                             10);

    std::ifstream ref_traj_data;
    ref_traj_data.open("/home/vnv/asl_gremlin1/src/test_asl_gremlin/src/matlab_data/reference_trajectory.txt",
                        std::ifstream::in);
    
    std::string first_line;
    std::getline(ref_traj_data, first_line); 

    asl_gremlin_msgs::RefTraj ref_traj;
    double time_tmp;
    int msg_count = 0;

    ros::Rate loop_rate(10);

    while (ros::ok() && !ref_traj_data.eof())
    {
        ref_traj_data >> time_tmp >> ref_traj.x >> ref_traj.x_dot >> ref_traj.x_ddot;
        ref_traj_data >> ref_traj.y >> ref_traj.y_dot >> ref_traj.y_ddot >> ref_traj.theta;

        ref_traj.theta_dot = 0.0; ref_traj.theta_ddot = 0.0;

        ref_traj.header.seq = msg_count;
        ref_traj.header.stamp = ros::Time::now();

        ref_traj_pub.publish(ref_traj);

        ++msg_count;

        loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}
