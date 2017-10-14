#ifndef _controller_BACKSTEPPINGCONTROLLER_H_
#define _controller_BACKSTEPPINGCONTROLLER_H_

#include "ControllerBase.h"
#include <array>
#include <algorithm>
#include <ros/ros.h>
#include <cmath>
#include <controller/controllerGainSetConfig.h>
#include "controller_utilities.h"
#include <dynamic_reconfigure/server.h>
#include <asl_gremlin_msgs/MotorAngVel.h>

using namespace controller;

#define SIGN(x) (x>0?1:(x==0?0:-1))

namespace controller{

template<typename ref_state_type, typename act_state_type>
class BackSteppingController : 
                        public ControllerBase<ref_state_type, act_state_type>{

    public:
        BackSteppingController(ros::NodeHandle&);
        ~BackSteppingController(){
            delete wheel_angular_vel_;
        }

        void calculate_control_action(const ref_state_type&, const act_state_type&) override;
        asl_gremlin_msgs::MotorAngVel* get_control_action() override;

        void reset(){
            wheel_angular_vel_->wl = 0.0;
            wheel_angular_vel_->wr = 0.0;
        }

    private:
        int msg_count_ = 0;
        std::array<double,3> lambda_gains_{{0.2, 0.2, 5.0}};
        double lambda_x_ = 0.0, lambda_y_ = 0.0,
               lambda_theta_ = 5.0, lambda_thetaDot_ = 1.0;

        asl_gremlin_msgs::MotorAngVel* wheel_angular_vel_;

        double radius_of_wheel_ = 0.06858, vehicle_base_length_ = 0.3353,
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

    auto nh_namespace = ros::this_node::getNamespace();

    if (!nh.getParam(nh_namespace+"/wheel/radius",radius_of_wheel_))
    {
        ROS_WARN("Can't access parameter: /%s/wheel/radius, setting to 0.06858m",
                    nh_namespace.c_str());
    }

    if (!nh.getParam(nh_namespace+"/chassis/base_length",vehicle_base_length_))
    {
        ROS_WARN("Can't access parameter: /%s/chassis/base_length, setting to 0.3353m",
                    nh_namespace.c_str());
    }

    if (!nh.getParam(nh_namespace+"/wheel/max_angular_vel",max_wheel_angular_vel_))
    {
        ROS_WARN("Can't access parameter: /%s/wheel/max_angular_vel, setting to 12.5(rad/sec)",
                    nh_namespace.c_str());
    }

    wheel_angular_vel_ = new asl_gremlin_msgs::MotorAngVel();
    wheel_angular_vel_->wl = 0.0;
    wheel_angular_vel_->wr = 0.0;
}


template<typename ref_state_type, typename act_state_type>
void BackSteppingController<ref_state_type, act_state_type>::calculate_control_action(const ref_state_type& ref, const act_state_type& actual)
{
    double actual_hdg = actual.heading*M_PI/180.0;

    double error_x = actual.pose.point.x - ref.x;
    double error_y = actual.pose.point.y - ref.y;
    
    lambda_x_ = lambda_gains_[0]*std::fabs(std::log(std::fabs( (error_x/0.1) + 0.01)));
    lambda_y_ = lambda_gains_[1]*std::fabs(std::log(std::fabs( (error_y/0.1) + 0.01)));

    double x_act_dot_req = ref.x_dot  - lambda_x_*error_x;
    double y_act_dot_req = ref.y_dot  - lambda_y_*error_y;

    double theta_cmd = std::atan2(y_act_dot_req, x_act_dot_req);

    double error_theta = controller::delta_theta(actual_hdg, theta_cmd);
    
    //if (std::fabs(error_theta) >= 7*M_PI/180.0)
    { lambda_theta_ = lambda_gains_[2]*std::fabs(std::log(std::fabs( (error_theta/0.1) + 0.01))); }

    double vel_cmd = std::sqrt( x_act_dot_req*x_act_dot_req + y_act_dot_req*y_act_dot_req );

    
    double angular_vel_sum = (2/radius_of_wheel_)*vel_cmd,
           angular_vel_diff = 0.0;

    if ( vel_cmd <= 0.2 )
    { angular_vel_diff = -lambda_theta_ * controller::delta_theta(actual_hdg, ref.theta); }

    else
    {
        double theta_dot_req = (1/vel_cmd)*(std::cos(theta_cmd)*(ref.y_ddot - lambda_y_*(vel_cmd*std::sin(actual_hdg) - ref.y_dot)) - 
                                            std::sin(theta_cmd)*(ref.x_ddot - lambda_x_*(vel_cmd*std::cos(actual_hdg) - ref.x_dot)));

        angular_vel_diff = (vehicle_base_length_/radius_of_wheel_)*(lambda_thetaDot_*theta_dot_req - lambda_theta_*error_theta);
    }

    if ( std::fabs(angular_vel_diff) > 40.0 )
    { angular_vel_diff = SIGN(angular_vel_diff)*40.0; }

    angular_vel_sum = std::min<double>(angular_vel_sum, 40.0);

    wheel_angular_vel_->wl = 0.5*(angular_vel_sum - angular_vel_diff);
    wheel_angular_vel_->wr = 0.5*(angular_vel_sum + angular_vel_diff);

    wheel_angular_vel_->wl = std::min(max_wheel_angular_vel_, std::max(-max_wheel_angular_vel_, static_cast<double>(wheel_angular_vel_->wl)));
    wheel_angular_vel_->wr = std::min(max_wheel_angular_vel_, std::max(-max_wheel_angular_vel_, static_cast<double>(wheel_angular_vel_->wr)));
}

template<typename ref_state_type, typename act_state_type>
asl_gremlin_msgs::MotorAngVel* BackSteppingController<ref_state_type, act_state_type>::get_control_action()
{
    wheel_angular_vel_->header.seq       =  msg_count_;
    wheel_angular_vel_->header.stamp     =  ros::Time::now();
    wheel_angular_vel_->header.frame_id  =  "wheel frame";
    ++msg_count_;

    return wheel_angular_vel_;
}

template<typename ref_state_type, typename act_state_type>
void BackSteppingController<ref_state_type, act_state_type>::dynamic_reconfigure_gain_callback(controller::controllerGainSetConfig& config, uint32_t level)
{
    ROS_INFO("controller gains updated: lambda_x:= %f, lambda_y:= %f, lambda_theta:= %f",
             config.lambda_x, config.lambda_y, config.lambda_theta);
    lambda_gains_[0] = config.lambda_x;
    lambda_gains_[1] = config.lambda_y;
    lambda_gains_[2] = config.lambda_theta;
}

} // end namespace {controller}


#endif
