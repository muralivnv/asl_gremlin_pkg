#include <ros/ros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <string>
#include <utility_pkg/CmdArgParser.h>
#include <utility_pkg/str_manip.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controllerGainSet_client");
    ros::NodeHandle gain_nh;

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter lam_x;
    dynamic_reconfigure::DoubleParameter lam_y;
    dynamic_reconfigure::DoubleParameter lam_theta;
    dynamic_reconfigure::DoubleParameter lam_thetaDot;
    dynamic_reconfigure::Config conf;

    std::string nh_namespace(ros::this_node::getNamespace());
    if ( nh_namespace == "/" || nh_namespace == "//")
    { nh_namespace = "/asl_gremlin1"; }

    if (argc > 2)
    {
        try 
        {
            utility_pkg::CmdArgParser cmd_arg_parser(argc, argv);

            utility_pkg::CmdArgParser::option* opt = cmd_arg_parser.get_param("-lx");
            if (opt != nullptr)
            {
                if (opt->second == "")
                {
                    ROS_WARN("controllerGainSet_client: gain 'lambda_x' is not specified, so didn't updated");
                }
                else
                {
                    lam_x.name = "lambda_x";
                    lam_x.value = std::stod(opt->second);
                    conf.doubles.push_back(lam_x);
                }
            }

            opt = cmd_arg_parser.get_param("-ly");
            if (opt != nullptr)
            {
                if (opt->second == "")
                {
                    ROS_WARN("controllerGainSet_client: gain 'lambda_y' is not specified, so didn't updated");
                }
                else
                {
                    lam_y.name = "lambda_y";
                    lam_y.value = std::stod(opt->second);
                    conf.doubles.push_back(lam_y);
                }
            }

            opt = cmd_arg_parser.get_param("-lth");
            if (opt != nullptr)
            {
                if (opt->second == "")
                {
                    ROS_WARN("controllerGainSet_client: gain 'lambda_theta' is not specified, so didn't updated");
                }
                else
                {
                    lam_theta.name = "lambda_theta";
                    lam_theta.value = std::stod(opt->second);
                    conf.doubles.push_back(lam_theta);
                }
            }
            
            opt = cmd_arg_parser.get_param("-lthD");
            if (opt != nullptr)
            {
                if (opt->second == "")
                {
                    ROS_WARN("controllerGainSet_client: gain 'lambda_thetaDot' is not specified, so didn't updated");
                }
                else
                {
                    lam_theta.name = "lambda_thetaDot";
                    lam_theta.value = std::stod(opt->second);
                    conf.doubles.push_back(lam_thetaDot);
                }
            }
            
            srv_req.config = conf;
            if (srv_req.config.doubles.empty())
            {
                throw "incorrect flags specified, gains didn't updated";
            }

            if(ros::service::call(nh_namespace+"/backstepping_controller/set_parameters",srv_req, srv_resp) )
            {
                ROS_INFO("controllerGainSet_client: controller gains updated successfully");
            }
            else
            {
                ROS_ERROR("controllerGainSet_client: controller gains didn't updated, try again!"
                        "\n \t \t \t \t OR \n check whether dynamic_reconfigure::Server ControllerGainSet is running");
            }
        }
        catch (const char* error_msg)
        {
            ROS_ERROR("controllerGainSet_client: %s",error_msg);
        }
    }
    else
    {
        ROS_ERROR("controllerGainSet_client: incorrect commandLine arguments");
    }
    return EXIT_SUCCESS;
}
