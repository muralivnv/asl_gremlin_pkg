#include <ros/ros.h>
#include <state_feedback/EncoderDataToOmega.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <utility_pkg/error_util.h>

#include <cmath>
#include <string>

using namespace state_feedback;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder_data_to_omega");
    ros::NodeHandle enco2w_nh;

    std::string actual_w_topic;
    if (!enco2w_nh.getParam("/asl_gremlin/state_feedback/encoder/ang_vel_topic", actual_w_topic))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/state_feedback/encoder/ang_vel_topic",
                                                __LINE__);
    }

    ros::Publisher enco2w_pub = enco2w_nh.advertise<asl_gremlin_msgs::MotorAngVel>(actual_w_topic,
                                                                                    100);

    EncoderDataToOmega encoder_data_to_omega(enco2w_nh);

    asl_gremlin_msgs::MotorAngVel motor_ang_vel;
    motor_ang_vel.header.frame_id = "none";

    ros::Rate loop_rate(10);

    int msg_count = 0;
    
    while(ros::ok())
    {
        encoder_data_to_omega.calculate_angular_velocities();
        motor_ang_vel.wl = encoder_data_to_omega.get_left_wheel_angular_vel();
        motor_ang_vel.wr = encoder_data_to_omega.get_right_wheel_angular_vel();
        motor_ang_vel.header.seq = msg_count;
        motor_ang_vel.header.stamp = ros::Time::now();
        
        enco2w_pub.publish(motor_ang_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
