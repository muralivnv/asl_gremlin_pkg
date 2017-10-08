#include <ros/ros.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feedbackSelect_client");
    ros::NodeHandle feedback_nh;

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::IntParameter feedback_select;
    dynamic_reconfigure::Config conf;

    if (argc == 2)
    {
        try 
        {
            feedback_select.name = "feedback";
            feedback_select.value = atoi(argv[1]);
            conf.ints.push_back(feedback_select);

            srv_req.config = conf;
            if(ros::service::call("/asl_gremlin1/feedback_selected/set_parameters",srv_req, srv_resp) )
            { ROS_INFO("Selected feedback is set"); }
            else
            { ROS_ERROR("Can't access topic '/asl_gremlin1/feedback_selected/set_parameters' "); }
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
