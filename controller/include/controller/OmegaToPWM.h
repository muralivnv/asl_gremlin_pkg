#ifndef _controller_OMEGATOPWM_H_
#define _controller_OMEGATOPWM_H_

#include <vector>
#include <ros/ros.h>
#include <utility_pkg/custom_algorithms.h>
#include <utility_pkg/error_util.h>
#include <asl_gremlin_msgs/MotorPwm.h>
#include <asl_gremlin_msgs/MotorAngVel.h>

namespace controller{

class OmegaToPWM{
    asl_gremlin_msgs::MotorAngVel* omega_cmd_;
    asl_gremlin_msgs::MotorPwm* pwm_cmd_;

    std::vector<double> pwm_lookup_, omega_lookup_;
    ros::Subscriber omega_sub_;

    public:
        OmegaToPWM(ros::NodeHandle&);
        ~OmegaToPWM();
        asl_gremlin_msgs::MotorPwm* convert_omega_to_pwm(double);
        void ang_vel_callback(const asl_gremlin_msgs::MotorAngVel::ConstPtr);

};

} // end namespace {controller}


#endif
