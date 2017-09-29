#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <cassert>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_publish_encoder_ticks");
    ros::NodeHandle nh;

    ros::Publisher right_ticks_publish = nh.advertise<std_msgs::Float64MultiArray>(ros::this_node::getNamespace()+"/state_feedback/encoder_right_timeStamped",
                                                        10);
    ros::Publisher left_ticks_publish  = nh.advertise<std_msgs::Float64MultiArray> (ros::this_node::getNamespace()+"/state_feedback/encoder_left_timeStamped",
                                                        10);
    
    std::ifstream file;
    file.open("/home/vnv/asl_gremlin1/src/test_asl_gremlin/src/matlab_data/encoder_data.txt",
                    std::ifstream::in);

    std::string str;
    std::getline(file, str);
    
    ros::Rate loop_rate(10);
    std_msgs::Float64MultiArray right_ticks, left_ticks;
    
    double left_time = 0.0, left_msg_count = 0.0, left_ticks_tmp = 0.0;
    double right_time = 0.0, right_msg_count = 0.0, right_ticks_tmp = 0.0;

    while(ros::ok() && !file.eof())
    {
        right_ticks.data.clear();
        left_ticks.data.clear();

        file >> left_time >> left_msg_count >> left_ticks_tmp;
        file >> right_time >> right_msg_count >> right_ticks_tmp;

        right_ticks.data.push_back(right_ticks_tmp);
        right_ticks.data.push_back(right_time);
        right_ticks.data.push_back(right_msg_count);

        left_ticks.data.push_back(left_ticks_tmp);
        left_ticks.data.push_back(left_time);
        left_ticks.data.push_back(left_msg_count);

        right_ticks_publish.publish(right_ticks);
        left_ticks_publish.publish(left_ticks);

        loop_rate.sleep();
    }
    file.close();

    return EXIT_SUCCESS;
}
