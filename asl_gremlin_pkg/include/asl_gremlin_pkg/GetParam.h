#ifndef _asl_gremlin_pkg_GETPARAM_H_
#define _asl_gremlin_pkg_GETPARAM_H_

#include <string>
#include <ros/ros.h>

namespace asl_gremlin_pkg{

void throw_warn(const std::string& , int);
void throw_error_and_shutdown(const std::string&, int);

template<typename T>
struct helper_struct{
    static T helper_get_param_with_shutdown(ros::NodeHandle& nh,
                                            const std::string& const_param_name,
                                            int line_num)
    {
        auto node_namespace = ros::this_node::getNamespace();
        if (node_namespace[0] == '/' && node_namespace[1] == '/')
        { node_namespace.erase(0,1); }

        auto param_name = node_namespace + const_param_name;

        T param_value;
        if(!nh.getParam(param_name, param_value))
        { asl_gremlin_pkg::throw_error_and_shutdown(param_name, line_num); }
        else
        { return param_value; }
    }

    static T helper_get_param_with_warn(ros::NodeHandle& nh,
                                        const std::string& const_param_name,
                                        int line_num)
    {
        auto node_namespace = ros::this_node::getNamespace();
        if (node_namespace[0] == '/' && node_namespace[1] == '/')
        { node_namespace.erase(0,1); }

        auto param_name = node_namespace + const_param_name;

        T param_value;
        if(!nh.getParam(param_name, param_value))
        { asl_gremlin_pkg::throw_warn(param_name, line_num); }
        else
        { return param_value; }
    }
};

template<>
struct helper_struct<std::string>{
    static std::string helper_get_param_with_shutdown(ros::NodeHandle& nh,
                                            const std::string& const_param_name,
                                            int line_num)
    {
        auto node_namespace = ros::this_node::getNamespace();
        if (node_namespace[0] == '/' && node_namespace[1] == '/')
        { node_namespace.erase(0,1); }

        auto param_name = node_namespace + const_param_name;

        std::string param_value;
        if(!nh.getParam(param_name, param_value))
        { asl_gremlin_pkg::throw_error_and_shutdown(param_name, line_num); }
        else
        { return node_namespace+param_value; }
    }

    static std::string helper_get_param_with_warn(ros::NodeHandle& nh,
                                        const std::string& const_param_name,
                                        int line_num)
    {
        auto node_namespace = ros::this_node::getNamespace();
        if (node_namespace[0] == '/' && node_namespace[1] == '/')
        { node_namespace.erase(0,1); }

        auto param_name = node_namespace + const_param_name;

        std::string param_value;
        if(!nh.getParam(param_name, param_value))
        { asl_gremlin_pkg::throw_warn(param_name, line_num); }
        else
        { return node_namespace+param_value; }
    }
};

template<typename T>
T GetParam_with_shutdown(ros::NodeHandle& nh, 
                        const std::string& const_param_name, 
                        int line_num)
{
    return helper_struct<T>::helper_get_param_with_shutdown
                             (nh, const_param_name, line_num);
}

template<typename T>
T GetParam_with_warn(ros::NodeHandle& nh, const std::string& const_param_name, int line_num)
{
    return helper_struct<T>::helper_get_param_with_warn
                             (nh, const_param_name, line_num);
}



} // end namespace {asl_gremlin_pkg}



#endif
