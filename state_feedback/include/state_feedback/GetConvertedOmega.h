#ifndef _state_feedback_GETCONVERTEDOMEGA_H_
#define _state_feedback_GETCONVERTEDOMEGA_H_

#include <ros/ros.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <utility_pkg/error_util.h>

namespace state_feedback {

class GetConvertedOmega{
    asl_gremlin_msgs::MotorAngVel* act_ang_vel_;

    ros::Subscriber omega_sub_;
    public:
        GetConvertedOmega(ros::NodeHandle&);
        
        ~GetConvertedOmega(){
            delete act_ang_vel_;
        }

        void encoder_to_w_callback(const asl_gremlin_msgs::MotorAngVel::ConstPtr);
        asl_gremlin_msgs::MotorAngVel* get_ang_vel();
};

} //end namespace {state_feedback}
#endif
