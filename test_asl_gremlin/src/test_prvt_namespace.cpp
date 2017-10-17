#include <boost/shared_ptr.hpp>
#include <iostream>
#include <geometry_msgs/PoseWithCovariance.h>
#include <chrono>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#define TIME_NOW std::chrono::system_clock::now()

void test_callback(const std_msgs::Bool::ConstPtr& msg)
{ }

int main(int argc, char** argv)
{
    ros::init(argc,argv,"test_ptr_assign");

    ros::NodeHandle nh;

    boost::shared_ptr<geometry_msgs::PoseWithCovariance> PosePtr, PosePtr2(new geometry_msgs::PoseWithCovariance());
    
    std::unique_ptr<geometry_msgs::PoseWithCovariance> PoseUPtr;
    auto start = TIME_NOW;
    auto end = TIME_NOW;

    ros::Publisher test_pub = nh.advertise<std_msgs::Bool>("test_pub",100);
    ros::Subscriber test_sub = nh.subscribe<std_msgs::Bool>("test_sub",100,test_callback);

    double radius = 0.0;
    if (nh.getParam("wheel/radius", radius))
    { ROS_INFO("HELL YEAH, %f", radius); }
    
    double total_elapsed = 0.0;
    std::chrono::duration<double> elapsed = end - start;

    ROS_INFO("DONE-----");
    ros::Rate loop_rate(10);
    std_msgs::Bool pub_data;
    pub_data.data = false;

    int count = 0;
    while (ros::ok())
    {
        start = TIME_NOW;
        
        PoseUPtr.reset(new geometry_msgs::PoseWithCovariance(*PosePtr2));
        end = TIME_NOW;
       
        elapsed = end - start;

        total_elapsed += elapsed.count();

        test_pub.publish(pub_data);
        loop_rate.sleep();
        ros::spinOnce();
        ++count;
    }

    std::cout << "elapsed: " << total_elapsed/count;

}
