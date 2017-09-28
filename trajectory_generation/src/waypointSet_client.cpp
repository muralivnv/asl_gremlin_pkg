#include <ros/ros.h>
#include <dynamic_reconfigure/StrParameter.h>
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
    dynamic_reconfigure::StrParameter X_wp;
    dynamic_reconfigure::StrParameter Y_wp;
    dynamic_reconfigure::Config conf;

    std::string waypoint_server_topic_name;
    waypoint_server_topic_name = asl_gremlin_pkg::GetParam_with_shutdown(wp_nh,"/trajectory/waypoint_server_topic",
                                                                                __LINE__);
	try{
	    if (argc > 2)
	    {
            utility_pkg::CmdArgParser cmd_arg_parser(argc, argv);

            utility_pkg::CmdArgParser::option* opt = cmd_arg_parser.get_param("-x");
	        if (opt != nullptr)
	        {
	            if (opt->second == "")
	            {
	                throw "X_component of waypoints not specified";
	            }
	            else
	            {
	                X_wp.name = "X_waypoint";
	                X_wp.value = opt->second;
	                conf.strs.push_back(X_wp);
	            }
	        }
	        opt = cmd_arg_parser.get_param("-y");
	        if (opt != nullptr)
	        {
	            if (opt->second == "")
	            {
	                throw "Y_component of waypoints not specified";
	            }
	            else
	            {
	                Y_wp.name = "Y_waypoint";
	                Y_wp.value = opt->second;
	                conf.strs.push_back(Y_wp);
	            }
	        }

	        srv_req.config = conf;
	        if (conf.strs.size() != 2)
	        {
	            throw "incorrect flags or one waypoint component(X/Y) didn't specified";
	        }
	        
	        if(ros::service::call(waypoint_server_topic_name, srv_req, srv_resp) )
	        {
	            ROS_INFO("waypoint_set_client: waypoints have been sent successfully");
	        }
	        else
	        {
	            ROS_ERROR("waypoint_set_client: waypoints didn't reached the server, try again! "
	                    "OR check whether dynamic_reconfigure::Server waypointSet is running");
	        }
	    }
	    else
	    {
	        throw "insufficient command line arguments";
	    }
	}
	catch (const char* msg)
	{
	    ROS_ERROR("waypoint_set_client: %s\n"
	                "\t\t\t\tuse syntax: -x string -y string", msg);
	}   

	return EXIT_SUCCESS;
}
