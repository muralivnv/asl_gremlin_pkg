#include <state_feedback/GetConvertedOmega.h>

using namespace state_feedback;

GetConvertedOmega::GetConvertedOmega(ros::NodeHandle& nh)
{
    act_ang_vel_ = new asl_gremlin_msgs::MotorAngVel();

    std::string ang_vel_topic;
    if(!nh.getParam("/asl_gremlin/state_feedback/encoder/ang_vel_topic",ang_vel_topic))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/state_feedback/encoder/ang_vel_topic",__LINE__);
    }

    omega_sub_ = nh.subscribe(ang_vel_topic,
                                100,
                                &GetConvertedOmega::encoder_to_w_callback,
                                this);
    ros::spinOnce();

}

void GetConvertedOmega::encoder_to_w_callback(const asl_gremlin_msgs::MotorAngVel::ConstPtr data)
{
    act_ang_vel_->header = data->header;
    act_ang_vel_->wl = data->wl;
    act_ang_vel_->wr = data->wr;
}

asl_gremlin_msgs::MotorAngVel*
GetConvertedOmega::get_ang_vel()
{
    return act_ang_vel_;
}
