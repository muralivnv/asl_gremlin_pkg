#include <state_feedback/FeedbackSelected.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feedback_selected");
    
    ros::NodeHandle feedback_nh;

    state_feedback::FeedbackSelected<3> feedback_method(feedback_nh);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        feedback_method.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

}
