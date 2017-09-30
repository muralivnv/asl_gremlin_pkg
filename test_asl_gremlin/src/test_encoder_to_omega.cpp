#include <ros/ros.h>
#include <state_feedback/EncoderDataToOmega.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <asl_gremlin_pkg/GetParam.h>

#include <cmath>
#include <string>
//#include <cassert>
//#include <fstream>

using namespace state_feedback;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_encoder_to_omega");
    ros::NodeHandle enco2w_nh;
  //  std::ofstream out_file("/home/vnv/asl_gremlin1/src/test_asl_gremlin/src/converted_encoder_to_omega.dat");
    
  //  assert(out_file.is_open());

    std::string actual_w_topic("/asl_gremlin1/state_feedback/encoder/actual_ang_vel");

    ros::Publisher enco2w_pub = enco2w_nh.advertise<asl_gremlin_msgs::MotorAngVel>(actual_w_topic,
                                                                                    100);
    EncoderDataToOmega encoder_data_to_omega(enco2w_nh);

    asl_gremlin_msgs::MotorAngVel motor_ang_vel;
    motor_ang_vel.header.frame_id = "none";

    ros::Rate loop_rate(5);

    int msg_count = 0;
    ros::spinOnce();
    
    while(ros::ok())
    {
        encoder_data_to_omega.calculate_angular_velocities();
        motor_ang_vel.wl = encoder_data_to_omega.get_left_wheel_angular_vel();
        motor_ang_vel.wr = encoder_data_to_omega.get_right_wheel_angular_vel();
        motor_ang_vel.header.seq = msg_count;
        motor_ang_vel.header.stamp = ros::Time::now();
    //    out_file << motor_ang_vel.wl << "   " << motor_ang_vel.wr << '\n';
        enco2w_pub.publish(motor_ang_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //out_file.close();
}
