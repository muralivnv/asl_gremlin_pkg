#include <asl_gremlin_pkg/GetParam.h>

void asl_gremlin_pkg::throw_warn(const std::string& param_name, int line_num)
{
    ROS_WARN("%s: At line [%d] ~ can't access param-> %s",
                ros::this_node::getName().c_str(),
                line_num,
                param_name.c_str());
}

void asl_gremlin_pkg::throw_error_and_shutdown(const std::string& param_name, int line_num)
{
    ROS_ERROR("%s: At line [%d] ~ can't access param-> %s", 
                ros::this_node::getName().c_str(),
                line_num,
                param_name.c_str());

    ROS_ERROR("Shutting down node: %s", 
            ros::this_node::getName().c_str());

    ros::shutdown();
}
