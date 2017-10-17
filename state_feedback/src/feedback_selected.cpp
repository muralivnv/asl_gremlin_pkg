#include <state_feedback/FeedbackSelected.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feedback_selected");
    ROS_INFO("Initialized:= %s",ros::this_node::getName().c_str());

    ros::NodeHandle feedback_nh;

    state_feedback::FeedbackSelected<3> feedback_method(feedback_nh);

    double rate = 10.0;

    if (!feedback_nh.getParam("sim/rate", rate))
    { ROS_WARN("Unable access parameter /%s/sim/rate, setting rate as 10Hz",
                ros::this_node::getNamespace().c_str()); }

    ros::Rate loop_rate(rate);

    
    while(ros::ok())
    {
        feedback_method.get_gps_data();
        feedback_method.get_enco_data();
        feedback_method.publish();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
