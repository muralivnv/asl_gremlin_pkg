/**
 * @brief Select necessary feedback node
 * @file feedbackSelect_client.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <ros/ros.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <utility_pkg/utilities.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feedbackSelect_client");
    ros::NodeHandle feedback_nh;

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter feedback_select;
    dynamic_reconfigure::Config conf;

    std::string nh_namespace(ros::this_node::getNamespace());
    if ( nh_namespace == "/" || nh_namespace == "//")
    { 
        nh_namespace = utility_pkg::get_robot_name(argv);
        if (nh_namespace == "")
        {nh_namespace = "/asl_gremlin1";}
        else
        {nh_namespace = "/"+nh_namespace;}
    }

    if (argc == 2)
    {
        try 
        {
            feedback_select.name = "feedback";
            feedback_select.value = atoi(argv[1]);
            conf.ints.push_back(feedback_select);

            srv_req.config = conf;
            if(ros::service::call(nh_namespace+"/feedback_selected/set_parameters",srv_req, srv_resp) )
            { ROS_INFO("Selected feedback is set"); }
            else
            { ROS_ERROR("Can't access topic %s/feedback_selected/set_parameters' ",nh_namespace.c_str()); }
        }
        catch (const char* error_msg)
        {
            ROS_ERROR("feedbackSelect_client: %s",error_msg);
        }
    }
    else
    {
        ROS_ERROR("use parameter '0': for GPS+compass | '1': for Encoder+compass | '2': for only GPS");
    }
    return EXIT_SUCCESS;
}
