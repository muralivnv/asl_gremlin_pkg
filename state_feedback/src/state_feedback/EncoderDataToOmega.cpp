#include <state_feedback/EncoderDataToOmega.h>

using namespace state_feedback;
using namespace utility_pkg;

EncoderDataToOmega::EncoderDataToOmega(ros::NodeHandle& nh)
{
    if(!nh.getParam("/asl_gremlin/motor/left_encoder_ticks_per_meter",
                        encoder_left_ticks_per_meter_))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/motor/left_encoder_ticks_per_meter",
                                    __LINE__);
    }
    if(!nh.getParam("/asl_gremlin/motor/right_encoder_ticks_per_meter",
                        encoder_right_ticks_per_meter_))
    {
        utility_pkg::throw_error_and_shutdown("encoder/right_wheel_ticks_per_meter",
                                    __LINE__);
    }
    if(!nh.getParam("/asl_gremlin/wheel/radius",
                        radius_of_wheel_))

    {
        utility_pkg::throw_warn("encoder/radius_of_wheel",
                                    __LINE__);
        ROS_WARN("Setting radius_of_wheel as 0.6858m ");
    }

    std::string encoder_left_topic, encoder_right_topic;

    if(!nh.getParam("/asl_gremlin/state_feedback/encoder/left_timeStamped_topic",
                        encoder_left_topic))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/state_feedback/encoder/left_timeStamped_topic",
                                    __LINE__);
    }

    if(!nh.getParam("/asl_gremlin/state_feedback/encoder/right_timeStamped_topic", 
                        encoder_right_topic))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/state_feedback/encoder/right_timeStamped_topic",
                                    __LINE__);
    }

    left_wheel_data_  = new asl_gremlin_pkg::SubscribeTopic<std_msgs::Float64MultiArray>(nh, encoder_left_topic, 150);
    right_wheel_data_ = new asl_gremlin_pkg::SubscribeTopic<std_msgs::Float64MultiArray>(nh, encoder_right_topic, 150);

    ros::spinOnce();

    update_encoder_starting_values();
}

EncoderDataToOmega::~EncoderDataToOmega()
{
    delete left_wheel_data_;
    delete right_wheel_data_;
}


void EncoderDataToOmega::update_encoder_starting_values()
{
    prev_left_encoder_ticks_ = (left_wheel_data_->get_data())->data[0];
    prev_left_encoder_time_  = (left_wheel_data_->get_data())->data[1];

    prev_right_encoder_ticks_ = (right_wheel_data_->get_data())->data[0];
    prev_right_encoder_time_  = (right_wheel_data_->get_data())->data[1];
}

void EncoderDataToOmega::update_delta_ticks()
{
   double left_encoder_ticks_now = (left_wheel_data_->get_data())->data[0];
   double left_encoder_time_now =  (left_wheel_data_->get_data())->data[1];

   delta_left_encoder_ticks_ = left_encoder_ticks_now - prev_left_encoder_ticks_;
   delta_left_encoder_time_  = left_encoder_time_now  - prev_left_encoder_time_;

   prev_left_encoder_ticks_ = left_encoder_ticks_now;
   prev_left_encoder_time_ = left_encoder_time_now;


   double right_encoder_ticks_now = (right_wheel_data_->get_data())->data[0];
   double right_encoder_time_now =  (right_wheel_data_->get_data())->data[1];

   delta_right_encoder_ticks_ = right_encoder_ticks_now - prev_right_encoder_ticks_;
   delta_right_encoder_time_  = right_encoder_time_now  - prev_right_encoder_time_;

   prev_right_encoder_ticks_ = right_encoder_ticks_now;
   prev_right_encoder_time_ = right_encoder_time_now;
}


void EncoderDataToOmega::calculate_angular_velocities()
{
    ros::spinOnce();
    update_delta_ticks();

    left_wheel_angular_velocity_ = (delta_left_encoder_ticks_)/
                    (encoder_left_ticks_per_meter_*radius_of_wheel_*delta_left_encoder_time_);

    right_wheel_angular_velocity_ = (delta_right_encoder_ticks_)/
                    (encoder_right_ticks_per_meter_*radius_of_wheel_*delta_right_encoder_time_);
}


double EncoderDataToOmega::get_left_wheel_angular_vel()
{
    return left_wheel_angular_velocity_;
}

double EncoderDataToOmega::get_right_wheel_angular_vel()
{
    return right_wheel_angular_velocity_;
}

