#include <asl_gremlin_pkg/StartSim.h>

using namespace asl_gremlin_pkg;

StartSim::StartSim(ros::NodeHandle& nh)
{
    start_sim_sub_ = nh.subscribe("/asl_gremlin/start_sim",
                                    10,
                                    &StartSim::start_sim_callback,
                                    this);
    ros::spinOnce();
}

void StartSim::start_sim_callback(const std_msgs::Bool::ConstPtr& data)
{
    data_ = data->data;
}

bool StartSim::start()
{
    return data_;
}
