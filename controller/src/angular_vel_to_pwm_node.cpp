#include <ros/ros.h>
#include <controller/OmegaToPWM.h>
#include <asl_gremlin_msgs/MotorPwm.h>

using namespace controller;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "angular_vel_to_pwm");

    ros::NodeHandle w2pwm_nh;
    
    OmegaToPWM omega_to_pwm(w2pwm_nh);

}

