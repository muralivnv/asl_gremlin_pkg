#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <state_feedback/ForwardEuler.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <std_msgs/Float64.h>
#include <utility_pkg/utilities.h>

#include <array>
#include <cmath>
#include <string>
#include <cassert>
#include <fstream>

#define deg2rad M_PI/180.0

using namespace state_feedback::numerical_diff;
using namespace state_feedback;

struct roverParam{
    double wl = 0.0, wr = 0.0;
    double r  = 0.06858, b = 0.3353;
};

std::array<double, 3> rover_kinematics(double time,
                                       std::array<double, 3> states,
                                        roverParam* params)
{
    double x_dot = (params->wl + params->wr)*0.5*params->r*std::cos(states[2]);
    double y_dot = (params->wl + params->wr)*0.5*params->r*std::sin(states[2]);
    double theta_dot = (params->r/params->b)*(params->wr - params->wl);


    return {x_dot, y_dot, theta_dot};
}


int main(int argc, char** argv)
{
    ros::init(argc, argv,"test_encoder_to_pose");
    ros::NodeHandle enco2w_nh;
    
    std::string tmp_str;
    std::ofstream out_file("/home/vnv/asl_gremlin1/src/test_asl_gremlin/src/cpp_output_data/pose_from_encoder.txt");
    assert(out_file.is_open());

    std::string encoder_pub_name, ang_vel_topic;
    if(!enco2w_nh.getParam("state_feedback/encoder/pose_topic",encoder_pub_name))
    { encoder_pub_name = "state_feedback/position_from_encoder"; }

    if(!enco2w_nh.getParam("state_feedback/encoder/ang_vel_topic", ang_vel_topic))
    { ang_vel_topic = "state_feedback/encoder/actual_ang_vel"; }

    ros::Publisher encoder_data_pub = enco2w_nh.advertise<geometry_msgs::PointStamped>(encoder_pub_name,10);

    asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::MotorAngVel> actual_angular_vel(enco2w_nh, ang_vel_topic);
    asl_gremlin_pkg::SubscribeTopic<std_msgs::Float64> compass_hdg(enco2w_nh, "/mavros/global_position/compass_hdg");

    roverParam params;
    geometry_msgs::PointStamped encoder_pose;

    ros::Rate loop_rate(10);

    std::array<double ,3> integrated_states{{0.0,0.0,0.0}};
    std::array<double, 3> initial_states{{0.0,0.0,0.0}};

    ros::spinOnce();

    initial_states[2] = (compass_hdg.get_data())->data * deg2rad;
    double t_initial = 0.0, t_final = t_initial + 0.1;

    int msg_count = 0;
    encoder_pose.point.x = initial_states[0];
    encoder_pose.point.y = initial_states[1];
    encoder_pose.point.z = 0;
    encoder_pose.header.seq = msg_count;
    encoder_pose.header.stamp = ros::Time::now();
    encoder_pose.header.frame_id = "local_tangent_enu";

    encoder_data_pub.publish(encoder_pose);
    asl_gremlin_msgs::MotorAngVel* actual_omega;

    out_file << "#pose_x " << "   " << "#pose_y" << "   " << "#theta_enu\n";
    forward_euler_step_size = 0.1;
    while(ros::ok())
    {
        actual_omega = actual_angular_vel.get_data();
        params.wl = actual_omega->wl;
        params.wr = actual_omega->wr;

        initial_states[2] = (compass_hdg.get_data())->data * deg2rad;

        integrated_states = forwardEuler_integration(rover_kinematics,
                                                    initial_states,
                                                    t_initial, t_final, &params);

        out_file  << integrated_states[0] << "    " << integrated_states[1] << "    " << initial_states[2]*180/M_PI << '\n';

        ++msg_count;
        encoder_pose.point.x = integrated_states[0];
        encoder_pose.point.y = integrated_states[1];
        encoder_pose.point.z = 0;
        encoder_pose.header.seq = msg_count;
        encoder_pose.header.stamp = ros::Time::now();
        encoder_data_pub.publish(encoder_pose);

        ros::spinOnce();
        loop_rate.sleep();

        initial_states = integrated_states;
        t_initial = t_final;
        t_final = t_final + 0.1;
    }
    out_file.close();
}
