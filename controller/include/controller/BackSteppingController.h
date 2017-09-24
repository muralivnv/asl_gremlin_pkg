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
    /* Removed for publication purposes */
}

template<typename ref_state_type, typename act_state_type>
std::array<double, 2> BackSteppingController<ref_state_type, act_state_type>::get_control_action()
{
    return wheel_angular_vel_;
}


} // end namespace {controller}
#endif
