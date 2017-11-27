/**
 * @brief waypointSet_client node
 * @file waypointSet_client.cpp
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
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <string>
#include <utility_pkg/CmdArgParser.h>
#include <asl_gremlin_pkg/GetParam.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_set_client");

    ros::NodeHandle wp_nh;

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::StrParameter X_wp, Y_wp;
    dynamic_reconfigure::BoolParameter cnctX, cnctY;
    dynamic_reconfigure::BoolParameter resetX, resetY;
    dynamic_reconfigure::BoolParameter removeX, removeY;
    dynamic_reconfigure::BoolParameter re_arrX, re_arrY;
    dynamic_reconfigure::Config conf;

    std::string nh_namespace(ros::this_node::getNamespace());
    if ( nh_namespace == "/" || nh_namespace == "//")
    { nh_namespace = "/asl_gremlin1"; }
    
    std::string waypoint_server_topic_name(nh_namespace+"/trajectory_generator/set_parameters");

	try{
	    if (argc > 2)
	    {
            utility_pkg::CmdArgParser cmd_arg_parser(argc, argv);

            utility_pkg::CmdArgParser::option* opt = cmd_arg_parser.get_param("-nx");
            resetX.name = "Reset_Xwp";
            resetX.value = false;
	        if (opt != nullptr)
	        {
	            if (opt->second == "")
	            { throw "data for X-waypoints not specified"; }
	            else
	            {
	            	resetX.value = true;
	                X_wp.name = "X_waypoint";
	                X_wp.value = opt->second;
	                conf.strs.push_back(X_wp);
	            }
	        }
	        conf.bools.push_back(resetX);

	        opt = cmd_arg_parser.get_param("-ax");
	        cnctX.name = "Concatenate_Xwp";
	        cnctX.value = false;
	        if (opt != nullptr)
	        {
	        	if (resetX.value)
	        	{throw "Specified both flags '-nx' and '-ax', rectify this";}
	        	
	        	if (opt->second == "")
	            { throw "data for X-waypoints not specified"; }
	            else
	            {
	            	cnctX.value = true;
	                X_wp.name = "X_waypoint";
	                X_wp.value = opt->second;
	                conf.strs.push_back(X_wp);
	            }
	        }
	        conf.bools.push_back(cnctX);

	        opt = cmd_arg_parser.get_param("-dx");
	        removeX.name = "Remove_Xwp";
	        removeX.value = false;
	        if (opt != nullptr)
	        {
	        	if (resetX.value)
	        	{throw "Specified both flags '-nx' and '-dx', rectify this";}
	        	if (cnctX.value)
	        	{throw "Specified both flags '-ax' and '-dx', rectify this";}
	        	
	        	if (opt->second == "")
	            { throw "data for X-waypoints not specified"; }
	            else
	            {
	            	removeX.value = true;
	                X_wp.name = "X_waypoint";
	                X_wp.value = opt->second;
	                conf.strs.push_back(X_wp);
	            }
	        }
	        conf.bools.push_back(removeX);

	        opt = cmd_arg_parser.get_param("-ny");
	        resetY.name = "Reset_Ywp";
            resetY.value = false;
	        if (opt != nullptr)
	        {
	            if (opt->second == "")
	            { throw "data for Y-waypoints not specified"; }
	            else
	            {
	                Y_wp.name = "Y_waypoint";
	                Y_wp.value = opt->second;
	                resetY.value = true;
	                conf.strs.push_back(Y_wp);
	            }
	        }
			conf.bools.push_back(resetY);

	        opt = cmd_arg_parser.get_param("-ay");
	        cnctY.name = "Concatenate_Ywp";
            cnctY.value = false;
	        if (opt != nullptr)
	        {
	        	if (resetY.value)
	        	{throw "Specified both flags '-ny' and '-ay', rectify this";}

	            if (opt->second == "")
	            { throw "data for  Y-waypoints not specified"; }
	            else
	            {
	                Y_wp.name = "Y_waypoint";
	                Y_wp.value = opt->second;
	                cnctY.value = true;
	                conf.strs.push_back(Y_wp);
	            }
	        }
	        conf.bools.push_back(cnctY);

	        opt = cmd_arg_parser.get_param("-dy");
	        removeY.name = "Remove_Ywp";
	        removeY.value = false;
	        if (opt != nullptr)
	        {
	        	if (resetY.value)
	        	{throw "Specified both flags '-ny' and '-dy', rectify this";}
	        	if (cnctY.value)
	        	{throw "Specified both flags '-ay' and '-dy', rectify this";}
	        	
	        	if (opt->second == "")
	            { throw "data for Y-waypoints not specified"; }
	            else
	            {
	            	removeY.value = true;
	                Y_wp.name = "Y_waypoint";
	                Y_wp.value = opt->second;
	                conf.strs.push_back(Y_wp);
	            }
	        }
	        conf.bools.push_back(removeY);
	        
	        srv_req.config = conf;
	        
	        if(ros::service::call(waypoint_server_topic_name, srv_req, srv_resp) )
	        { ROS_INFO("waypoint_set_client: waypoints have been sent successfully"); }
	        else
	        {
	            ROS_ERROR("waypoint_set_client: waypoints didn't reached the server, try again! "
	                    "OR check whether dynamic_reconfigure::Server waypointSet is running");
	        }
	    }
	    else
	    { throw "insufficient command line arguments"; }
	}
	catch (const char* msg)
	{
	    ROS_ERROR("waypoint_set_client: %s\n"
	                "\t\t\t\tuse syntax: -nx string -ny string", msg);
	}   
	return EXIT_SUCCESS;
}