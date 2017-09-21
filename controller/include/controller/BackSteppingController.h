#ifndef CONTROLLER__BACKSTEPPING_CONTROLLER_H
#define CONTROLLER__BACKSTEPPING_CONTROLLER_H

#include "ControllerBase.h"
#include <array>
#include <algorithm>
#include <ros/ros.h>
#include <cmath>
#include <controller/controllerGainSetConfig.h>
#include "controller_utilities.h"
#include <dynamic_reconfigure/server.h>
#include <asl_gremlin_msgs/MotorCmd.h>

using namespace controller;

#define sign(x) (x>0?1:(x==0?0:-1))

namespace controller{

template<typename ref_state_type, typename act_state_type>
class BackSteppingController : 
                        public ControllerBase<ref_state_type, act_state_type>{

        public:
            BackSteppingController(ros::NodeHandle&);

            void calculate_control_action(const ref_state_type&, const act_state_type&) override;
            std::array<double, 2> get_control_action() override;

        private:
            std::array<double,3> lambda_gains_{0.2, 0.2, 5.0};
            double lambda_x_ = 0.0, lambda_y_ = 0.0,
                   lambda_theta_ = 5.0, lambda_thetaDot_ = 1.0;

            std::array<double, 2> wheel_angular_vel_;

            double radius_of_wheel_ = 0.6858, vehicle_base_length_ = 0.3353,
                   max_wheel_angular_vel_ = 12.5; 

            dynamic_reconfigure::Server<controller::controllerGainSetConfig> dr_gain_srv_;
            dynamic_reconfigure::Server<controller::controllerGainSetConfig>::CallbackType fun_;
            void dynamic_reconfigure_gain_callback(controller::controllerGainSetConfig&, uint32_t);
};



template<typename ref_state_type, typename act_state_type>
BackSteppingController<ref_state_type, act_state_type>::BackSteppingController(ros::NodeHandle& nh)
{
    fun_ = boost::bind(&BackSteppingController::dynamic_reconfigure_gain_callback,
                        this, _1, _2);
    dr_gain_srv_.setCallback(fun_);

    if (!nh.getParam("/asl_gremlin/wheel/radius",radius_of_wheel_))
    {
        ROS_WARN("Can't access parameter: /asl_gremlin/wheel/radius, setting to 0.6858m");
    }

    if (!nh.getParam("/asl_gremlin/chassis/base_length",radius_of_wheel_))
    {
        ROS_WARN("Can't access parameter: /asl_gremlin/chassis/base_length, setting to 0.3353m");
    }

    if (!nh.getParam("/asl_gremlin/wheel/max_angular_vel",radius_of_wheel_))
    {
        ROS_WARN("Can't access parameter: /asl_gremlin/wheel/max_angular_vel, setting to 12.5(rad/sec)");
    }
}


template<typename ref_state_type, typename act_state_type>
void BackSteppingController<ref_state_type, act_state_type>::calculate_control_action(const ref_state_type& ref, const act_state_type& actual)
{
    double error_x = actual.x - ref.x;
    double error_y = actual.y - ref.y;
    
    lambda_x_ = lambda_gains_[0]*std::log( std::fabs(error_x)/0.1 + 0.01 );
    lambda_y_ = lambda_gains_[1]*std::log( std::fabs(error_y)/0.1 + 0.01 );

    double x_act_dot_req = ref.x_dot  - lambda_x_*error_x;
    double y_act_dot_req = ref.y_dot  - lambda_y_*error_y;

    double theta_cmd = std::atan2(y_act_dot_req, x_act_dot_req);

    double error_theta = controller::delta_theta(actual.theta, theta_cmd);
    
    if (std::fabs(error_theta) >= 7*M_PI/180.0)
        lambda_theta_ = lambda_gains_[2]*std::log( std::fabs(error_theta)/0.1 + 0.01 );

    double vel_cmd = std::sqrt( std::pow(x_act_dot_req,2) + std::pow(y_act_dot_req,2) );

    
    double angular_vel_sum = (2/radius_of_wheel_)*vel_cmd,
           angular_vel_diff = 0.0;

    if ( vel_cmd <= 0.2 )
    {
        angular_vel_diff = -lambda_theta_ * controller::delta_theta(actual.theta, ref.theta);
    }
    else
    {
        double theta_dot_req = (1/vel_cmd)*(std::cos(theta_cmd)*(ref.y_ddot - lambda_y_*(vel_cmd*std::sin(actual.theta) - ref.y_dot)) - 
                                            std::sin(theta_cmd)*(ref.x_ddot - lambda_x_*(vel_cmd*std::cos(actual.theta) - ref.x_dot)));

        angular_vel_diff = (vehicle_base_length_/radius_of_wheel_)*(lambda_thetaDot_*theta_dot_req - lambda_theta_*error_theta);
    }

    if (std::fabs(angular_vel_diff) > 40 )
    {
        angular_vel_diff = sign(angular_vel_diff)*40;
    }

    angular_vel_sum = std::min(angular_vel_sum, 40.0);

    wheel_angular_vel_[0] = 0.5*(angular_vel_sum - angular_vel_diff);
    wheel_angular_vel_[1] = 0.5*(angular_vel_sum + angular_vel_diff);

    wheel_angular_vel_[0] = std::min(max_wheel_angular_vel_, std::max(-max_wheel_angular_vel_, wheel_angular_vel_[0]));
    wheel_angular_vel_[1] = std::min(max_wheel_angular_vel_, std::max(-max_wheel_angular_vel_, wheel_angular_vel_[1]));
}

template<typename ref_state_type, typename act_state_type>
std::array<double, 2> BackSteppingController<ref_state_type, act_state_type>::get_control_action()
{
    return wheel_angular_vel_;
}


} // end namespace
#endif
