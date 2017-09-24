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
    ros::Rate loop_rate(10);
    ros::spinOnce();

    std::string pwm_pub_topic;

    if (!w2pwm_nh.getParam("/asl_gremlin/controller/cmd_pwm_topic",
                           pwm_pub_topic))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/controller/cmd_pwm_topic",
                                              __LINE__);
    }

    ros::Publisher pwm_pub = w2pwm_nh.advertise<asl_gremlin_msgs::MotorPwm>
                                                        (pwm_pub_topic,20);

    while(ros::ok())
    {
        pwm_pub.publish(*(omega_to_pwm.convert_omega_to_pwm()));
        ros::spinOnce();
        loop_rate.sleep();
    }

}

