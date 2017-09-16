#include <state_feedback/EncoderDataToOmega.h>

using namespace state_feedback;

void EncoderDataToOmega::left_encoder_callback(const std_msgs::Float64MultiArray::ConstPtr data)
{
   double left_encoder_ticks_now = data->data[0];
   double left_encoder_time_now = data->data[1];

   delta_left_encoder_ticks_ = left_encoder_ticks_now - prev_left_encoder_ticks_;
   delta_left_encoder_time_  = left_encoder_time_now  - prev_left_encoder_time_;

   prev_left_encoder_ticks_ = left_encoder_ticks_now;
   prev_left_encoder_time_ = left_encoder_time_now;

}

void EncoderDataToOmega::right_encoder_callback(const std_msgs::Float64MultiArray::ConstPtr data)
{
   double right_encoder_ticks_now = data->data[0];
   double right_encoder_time_now = data->data[1];

   delta_right_encoder_ticks_ = right_encoder_ticks_now - prev_right_encoder_ticks_;
   delta_right_encoder_time_  = right_encoder_time_now  - prev_right_encoder_time_;

   prev_right_encoder_ticks_ = right_encoder_ticks_now;
   prev_right_encoder_time_ = right_encoder_time_now;
}

void EncoderDataToOmega::calculate_angular_velocities()
{
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
