#ifndef STATE_FEEDBACK__ENCODER_DATA_TO_OMEGA_H
#define STATE_FEEDBACK__ENCODER_DATA_TO_OMEGA_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>

namespace state_feedback{

class EncoderDataToOmega{
    double left_wheel_angular_velocity_ = 0.0,
            right_wheel_angular_velocity_ = 0.0;

    double encoder_left_ticks_per_meter_ = 0.0,
           encoder_right_ticks_per_meter_ = 0.0;
    
    double prev_left_encoder_time_ = 0.0,
           prev_right_encoder_time_ = 0.0;

    double prev_left_encoder_ticks_ = 0.0,
           prev_right_encoder_ticks_ = 0.0;

    double delta_left_encoder_ticks_ = 0.0,
           delta_right_encoder_ticks_ = 0.0;

    double delta_left_encoder_time_ = 0.0,
           delta_right_encoder_time_ = 0.0;
    
    double radius_of_wheel_ = 0.6858;
 
    ros::Subscriber left_wheel_subscriber_,
                    right_wheel_subscriber_;
    public:
        EncoderDataToOmega(ros::NodeHandle& nh){
            if(!nh.getParam("/asl_gremlin/motor/left_encoder_ticks_per_meter", encoder_left_ticks_per_meter_))
            {
                ROS_ERROR("Can't access Param: '/asl_gremlin/motor/left_encoder_ticks_per_meter'");
                ROS_ERROR("Shutting Down 'Encoder_To_Pose_Node'");
                ros::shutdown();
            }
            if(!nh.getParam("/asl_gremlin/motor/right_encoder_ticks_per_meter", encoder_right_ticks_per_meter_))
            {
                ROS_ERROR("Can't access Param: 'encoder/right_wheel_ticks_per_meter'");
                ROS_ERROR("Shutting Down 'Encoder_To_Pose_Node'");
                ros::shutdown();
            }
            if(!nh.getParam("/asl_gremlin/wheel/radius", radius_of_wheel_))
            {
                ROS_ERROR("Can't access Param: 'encoder/radius_of_wheel'");
                ROS_WARN("Setting radius_of_wheel as 0.6858m ");
            }

            std::string encoder_left_topic, encoder_right_topic;

            if(!nh.getParam("/asl_gremlin/state_feedback/encoder/left_timeStamped_topic", encoder_left_topic))
            {
                ROS_ERROR("Can't access Param: /asl_gremlin/state_feedback/encoder/left_timeStamped_topic");
                ros::shutdown();
            }

            if(!nh.getParam("/asl_gremlin/state_feedback/encoder/right_timeStamped_topic", encoder_right_topic))
            {
                ROS_ERROR("Can't access Param: /asl_gremlin/state_feedback/encoder/right_timeStamped_topic");
                ros::shutdown();
            }
            left_wheel_subscriber_ = 
                            nh.subscribe(encoder_left_topic,
                                            10, &EncoderDataToOmega::left_encoder_callback,
                                            this);

            right_wheel_subscriber_ = 
                            nh.subscribe(encoder_right_topic,
                                            10, &EncoderDataToOmega::right_encoder_callback,
                                            this);
            ros::spinOnce();
        }

        void left_encoder_callback(const std_msgs::Float64MultiArray::ConstPtr);
        void right_encoder_callback(const std_msgs::Float64MultiArray::ConstPtr);
        void calculate_angular_velocities();
        double get_left_wheel_angular_vel();
        double get_right_wheel_angular_vel();

};

} // end namespace
#endif
