#include <ros/ros.h>
#include <controller/OmegaToPWM.h>
#include <asl_gremlin_msgs/MotorPwm.h>
#include <string>

using namespace controller;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "angular_vel_to_pwm");
    ros::NodeHandle w2pwm_nh;
    
    OmegaToPWM omega_to_pwm(w2pwm_nh);

    ros::spinOnce();

    std::string pwm_pub_topic;

    pwm_pub_topic = asl_gremlin_pkg::GetParam_with_shutdown<std::string>
                    (w2pwm_nh,"/controller/cmd_pwm_topic", __LINE__); 

    ros::Publisher pwm_pub = w2pwm_nh.advertise<asl_gremlin_msgs::MotorPwm>
                                                        (pwm_pub_topic,20);
    int rate = 10;
    if (!w2pwm_nh.getParam(ros::this_node::getNamespace()+"/sim/rate", rate))
    {
        ROS_WARN("Unable access parameter $robot_name/sim/rate, setting rate as 10Hz");
    }
    ros::Rate loop_rate(rate);
    
    while(ros::ok())
    {
        pwm_pub.publish(*(omega_to_pwm.convert_omega_to_pwm()));
        ros::spinOnce();
        loop_rate.sleep();
    }

}

