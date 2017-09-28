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

std::string asl_gremlin_pkg::GetParam_with_shutdown( ros::NodeHandle& nh, 
                                    const std::string& const_param_name, int line_num)
{
    auto node_namespace = ros::this_node::getNamespace();
    auto param_name = node_namespace + const_param_name;
    
    std::string topic_name;
    
    if(!nh.getParam(param_name, topic_name))
    {
        asl_gremlin_pkg::throw_error_and_shutdown(param_name, line_num); 
    }
    else
    { return topic_name; }
}

std::string asl_gremlin_pkg::GetParam_with_warn(ros::NodeHandle& nh, const std::string& const_param_name, int line_num)
{
    auto node_namespace = ros::this_node::getNamespace();
    auto param_name = node_namespace + const_param_name;
    
    std::string topic_name;
    
    if(!nh.getParam(param_name, topic_name))
    {
        asl_gremlin_pkg::throw_warn(param_name, line_num); 
    }
    else
    { return topic_name; }
}
