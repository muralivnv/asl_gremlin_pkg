#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <fstream>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_publish_compass_data");
    ros::NodeHandle nh;

    ros::Publisher cmp_hdg_pub = nh.advertise<std_msgs::Float64>("/mavros/global_position/compass_hdg",
                                                                 10);

    std::string tmp_string;
    std::ifstream cmp_hdg_data_file;
    cmp_hdg_data_file.open("/home/vnv/asl_gremlin1/src/test_asl_gremlin/src/matlab_data/compass_hdg_ENU.txt",
                                std::ifstream::in);
    
    std::getline(cmp_hdg_data_file, tmp_string);

    std_msgs::Float64 cmp_hdg_pub_data;
    double time = 0.0;
    ros::Rate loop_rate(10);

    while (ros::ok() && !cmp_hdg_data_file.eof())
    {
        cmp_hdg_data_file >> time >> cmp_hdg_pub_data.data;
        
        cmp_hdg_pub.publish(cmp_hdg_pub_data);
        loop_rate.sleep();
    }

    cmp_hdg_data_file.close();
}
