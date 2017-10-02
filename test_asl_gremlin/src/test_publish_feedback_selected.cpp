#include <ros/ros.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <std_msgs/Float64.h>

#include <fstream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_publish_feedback_selected");
    ros::NodeHandle nh;

    ros::Publisher act_state_pub = nh.advertise<asl_gremlin_msgs::VehicleState>
                                ("/asl_gremlin1/state_feedback/selected_feedback",10);
    
    asl_gremlin_pkg::SubscribeTopic<std_msgs::Float64>
                        cmp_hdg(nh, "/mavros/global_position/compass_hdg");

    ros::Rate loop_rate(10);

    std::string first_line;
    std::ifstream act_gps_data;
    act_gps_data.open("/home/vnv/asl_gremlin1/src/test_asl_gremlin/src/matlab_data/actual_state.txt",
                    std::ifstream::in);
    std::getline(act_gps_data, first_line);

    double time_tmp = 0.0;
    int msg_count = 0;

    asl_gremlin_msgs::VehicleState act_gps;
    ros::spinOnce();

    while(ros::ok() && !act_gps_data.eof())
    {
        act_gps_data >> time_tmp >> act_gps.pose.point.x >> act_gps.pose.point.y ;
        act_gps.heading = (cmp_hdg.get_data())->data;

        act_gps.pose.header.seq = msg_count;
        act_gps.pose.header.stamp = ros::Time::now();
        ++msg_count;

        act_state_pub.publish(act_gps);
        ros::spinOnce();
        loop_rate.sleep();
    }

}
