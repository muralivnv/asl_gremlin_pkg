#include <controller/OmegaToPWM.h>

using namespace controller;
using namespace utility_pkg::custom_algorithms;

OmegaToPWM::OmegaToPWM(ros::NodeHandle& nh)
{
    omega_lookup_ = asl_gremlin_pkg::GetParam_with_shutdown<std::vector<double>>
                    (nh, "/motor/omega_to_pwm/omega", __LINE__);

    pwm_lookup_ = asl_gremlin_pkg::GetParam_with_shutdown<std::vector<double>>
                    (nh, "/motor/omega_to_pwm/pwm", __LINE__);

    if (omega_lookup_.size() != pwm_lookup_.size())
    {
        ROS_ERROR("Mismatch sizes, /motor/omega_to_pwm/pwm.size() != /motor/omega_to_pwm/omega.size()");
        ros::shutdown();
    }

    std::string ang_vel_topic;
    ang_vel_topic = asl_gremlin_pkg::GetParam_with_shutdown<std::string>
                    (nh, "/controller/cmd_angular_vel_topic",__LINE__);

    pwm_cmd_ = new asl_gremlin_msgs::MotorPwm();
    omega_cmd_ = new asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::MotorAngVel>(nh, ang_vel_topic,20);
}

OmegaToPWM::~OmegaToPWM()
{
    delete pwm_cmd_;
    delete omega_cmd_;
}

asl_gremlin_msgs::MotorPwm*
OmegaToPWM::convert_omega_to_pwm()
{
    ros::spinOnce();
    int left_idx = get_lower_index(omega_lookup_, (omega_cmd_->get_data())->wl);
    
    pwm_cmd_->pwm_l  = linear_interpolate(pwm_lookup_, omega_lookup_, left_idx, (omega_cmd_->get_data())->wl);
   
   int right_idx = get_lower_index(omega_lookup_, (omega_cmd_->get_data())->wr);
   pwm_cmd_->pwm_r  = linear_interpolate(pwm_lookup_, omega_lookup_, right_idx, (omega_cmd_->get_data())->wr);
   
   pwm_cmd_->header = (omega_cmd_->get_data())->header;

   return pwm_cmd_;
}

