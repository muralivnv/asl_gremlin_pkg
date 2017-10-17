#include <state_feedback/EncoderDataToOmega.h>

using namespace state_feedback;
using namespace asl_gremlin_pkg;

EncoderDataToOmega::EncoderDataToOmega(ros::NodeHandle& nh)
{
    encoder_left_ticks_per_meter_ = GetParam_with_shutdown<double>
                                    (nh, "motor/left_encoder_ticks_per_meter", __LINE__);

    encoder_right_ticks_per_meter_ = GetParam_with_shutdown<double>
                                    (nh, "motor/right_encoder_ticks_per_meter", __LINE__);

    radius_of_wheel_ = GetParam_with_warn<double>
                        (nh, "wheel/radius", __LINE__ );

    std::string encoder_left_topic, encoder_right_topic;
    encoder_left_topic = GetParam_with_shutdown<std::string>
                            (nh, "state_feedback/encoder/left_timeStamped_topic", __LINE__);

    encoder_right_topic = GetParam_with_shutdown<std::string>
                            (nh, "state_feedback/encoder/right_timeStamped_topic", __LINE__);

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
    if ( !(left_wheel_data_->get_data())->data.empty())
    {
        prev_left_encoder_ticks_ = (left_wheel_data_->get_data())->data[0];
        prev_left_encoder_time_  = (left_wheel_data_->get_data())->data[1];
    }

    if ( !(right_wheel_data_->get_data())->data.empty())
    {
        prev_right_encoder_ticks_ = (right_wheel_data_->get_data())->data[0];
        prev_right_encoder_time_  = (right_wheel_data_->get_data())->data[1];
    }
}

void EncoderDataToOmega::update_delta_ticks()
{
    if ( !(left_wheel_data_->get_data())->data.empty())
    {
        double left_encoder_ticks_now = (left_wheel_data_->get_data())->data[0];
        double left_encoder_time_now =  (left_wheel_data_->get_data())->data[1];

       delta_left_encoder_ticks_ = left_encoder_ticks_now - prev_left_encoder_ticks_;
       delta_left_encoder_time_  = left_encoder_time_now  - prev_left_encoder_time_;

       prev_left_encoder_ticks_ = left_encoder_ticks_now;
       prev_left_encoder_time_ = left_encoder_time_now;
    }

    if ( !(right_wheel_data_->get_data())->data.empty())
    {
        double right_encoder_ticks_now = (right_wheel_data_->get_data())->data[0];
        double right_encoder_time_now =  (right_wheel_data_->get_data())->data[1];

       delta_right_encoder_ticks_ = right_encoder_ticks_now - prev_right_encoder_ticks_;
       delta_right_encoder_time_  = right_encoder_time_now  - prev_right_encoder_time_;

       prev_right_encoder_ticks_ = right_encoder_ticks_now;
       prev_right_encoder_time_ = right_encoder_time_now;
    }
}


void EncoderDataToOmega::calculate_angular_velocities()
{
    ros::spinOnce();
    update_delta_ticks();

    if (delta_left_encoder_time_ != 0.0)
    {
        left_wheel_angular_velocity_ = (delta_left_encoder_ticks_)/
                        (encoder_left_ticks_per_meter_*radius_of_wheel_*delta_left_encoder_time_);
    }

    if (delta_right_encoder_time_ != 0.0)
    {
        right_wheel_angular_velocity_ = (delta_right_encoder_ticks_)/
                        (encoder_right_ticks_per_meter_*radius_of_wheel_*delta_right_encoder_time_);
    }
}


double EncoderDataToOmega::get_left_wheel_angular_vel()
{ return left_wheel_angular_velocity_; }

double EncoderDataToOmega::get_right_wheel_angular_vel()
{ return right_wheel_angular_velocity_; }

