#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <state_feedback/ForwardEuler.h>
#include <state_feedback/GetConvertedOmega.h>
#include <state_feedback/GetCompassHdg.h>
#include <array>
#include <cmath>

#define deg2rad M_PI/180.0

using namespace state_feedback::numerical_diff;
using namespace state_feedback;

struct roverParam{
    double wl = 0.0, wr = 0.0;
    double r  = 0.6858, b = 0.3353;
};

std::array<double, 3> rover_kinematics(double time,
                                       std::array<double, 3> states,
                                        roverParam params)
{
    std::array<double, 3> dx_dt;
    double x_dot = (params.wl + params.wr)*0.5*params.r*std::cos(states[2]);
    double y_dot = (params.wl + params.wr)*0.5*params.r*std::sin(states[2]);
    double theta_dot = (params.r/params.b)*(params.wr - params.wl);

    dx_dt[0] = x_dot; dx_dt[1] = y_dot; dx_dt[2] = theta_dot;

    return dx_dt;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv,"encoder_to_pose");
    ros::NodeHandle enco2w_nh;

    std::string encoder_pub_name;
    if (!enco2w_nh.getParam("/asl_gremlin/state_feedback/encoder/pose_topic", encoder_pub_name) )
    {
        ROS_ERROR("Can't access param /asl_gremlin/state_feedback/encoder/pose_topic");
        ros::shutdown();
    }
    ros::Publisher encoder_data_pub = enco2w_nh.advertise<geometry_msgs::PointStamped>(encoder_pub_name,10);

    GetConvertedOmega actual_angular_vel(enco2w_nh);
    GetCompassHdg compass_hdg(enco2w_nh);

    roverParam params;
    geometry_msgs::PointStamped encoder_pose;

    ros::Rate loop_rate(10);

    std::array<double ,3> integrated_states;
    std::array<double, 3> initial_states{0,0,0};

    ros::spinOnce();
    initial_states[2] = compass_hdg.data_ENU() * deg2rad;
    double t_initial = 0.0, t_final = t_initial + forward_euler_step_size;

    int msg_count = 0;
    encoder_pose.point.x = initial_states[0];
    encoder_pose.point.y = initial_states[1];
    encoder_pose.point.z = 0;
    encoder_pose.header.seq = msg_count;
    encoder_pose.header.stamp = ros::Time::now();
    encoder_pose.header.frame_id = "local_tangent_enu";

    encoder_data_pub.publish(encoder_pose);
    asl_gremlin_msgs::MotorAngVel* actual_omega;

    while(ros::ok())
    {
        actual_omega = actual_angular_vel.get_ang_vel();
        params.wl = actual_omega->wl;
        params.wr = actual_omega->wr;

        initial_states[2] = compass_hdg.data_ENU() * deg2rad;

        integrated_states = forwardEuler_integration(rover_kinematics,
                                                initial_states,
                                                t_initial, t_final, &params);

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
        t_final = t_final + forward_euler_step_size;
    }
}

