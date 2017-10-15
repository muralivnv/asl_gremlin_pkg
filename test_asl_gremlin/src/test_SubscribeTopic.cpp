#include <ros/ros.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>

using namespace asl_gremlin_msgs;
using namespace asl_gremlin_pkg;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_subscribetopic");
    ros::NodeHandle nh;

    SubscribeTopic<RefTraj>* ref_traj = new SubscribeTopic<RefTraj>(nh, "/asl_gremlin1/trajectory_generation/reference_trajectory");
    ros::Publisher ref_traj_pub = nh.advertise<RefTraj>("/asl_gremlin/trajectory_generation/test_poly_repub",
                                                         10);
    ros::spinOnce();
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ref_traj_pub.publish(*(ref_traj->get_data()));

        ros::spinOnce();
        loop_rate.sleep();
    }
}
