#include <ros/ros.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_traj_with_turning");
    ros::NodeHandle nh;

    ros::Publisher ref_traj_pub = nh.advertise<asl_gremlin_msgs::VehicleState>
                            ("/asl_gremlin1/state_feedback/selected_feedback",
                             10);
    
    asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::RefTraj> ref_traj
            (nh, "/asl_gremlin1/trajectory_generation/reference_trajectory");
    

    asl_gremlin_msgs::VehicleState vehicle_state;

    ros::Rate loop_rate(10);
    
    int msg_count = 0;
    while (ros::ok())
    {   
        vehicle_state.pose.point.x = (ref_traj.get_data())->x;
        vehicle_state.pose.point.y = (ref_traj.get_data())->y;
        vehicle_state.heading = (ref_traj.get_data())->theta * 180/M_PI;

        vehicle_state.pose.header.seq = msg_count;
        vehicle_state.pose.header.stamp = ros::Time::now();
        
        ref_traj_pub.publish(vehicle_state);
        
        ++msg_count;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
