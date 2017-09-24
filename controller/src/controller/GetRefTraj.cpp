#include <controller/GetRefTraj.h>

using namespace controller;
using namespace utility_pkg;

GetRefTraj::GetRefTraj(ros::NodeHandle& nh)
{
    std::string ref_traj_topic_name;
    if (!nh.getParam("/asl_gremlin/trajectory/publisher_topic",
                        ref_traj_topic_name))
    {
        throw_error_and_shutdown("/asl_gremlin/trajectory/publisher_topic",
                                    __LINE__);
    }

    ref_traj_sub_ = nh.subscribe(ref_traj_topic_name,20,
                                &GetRefTraj::ref_traj_callback,
                                this);

    ref_trajectory_ = new asl_gremlin_msgs::RefTraj();
}

GetRefTraj::~GetRefTraj()
{
    delete ref_trajectory_;
}

void GetRefTraj::ref_traj_callback(const asl_gremlin_msgs::RefTraj::ConstPtr data)
{
    ref_trajectory_->header     = data->header;
    ref_trajectory_->x          = data->x;
    ref_trajectory_->x_dot      = data->x_dot;
    ref_trajectory_->x_ddot     = data->x_ddot;
    ref_trajectory_->y          = data->y;
    ref_trajectory_->y_dot      = data->y_dot;
    ref_trajectory_->y_ddot     = data->y_ddot;
    ref_trajectory_->theta      = data->theta;
    ref_trajectory_->theta_dot  = data->theta_dot;
    ref_trajectory_->theta_ddot = data->theta_ddot;
}

asl_gremlin_msgs::RefTraj* 
GetRefTraj::get_ref_traj()
{
    return ref_trajectory_;
}

