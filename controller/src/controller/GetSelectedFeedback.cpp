#include <controller/GetSelectedFeedback.h>

using namespace controller;
using namespace utility_pkg;

GetSelectedFeedback::GetSelectedFeedback(ros::NodeHandle& nh)
{
    std::string act_state_topic;

    if (!nh.getParam("/asl_gremlin/state_feedback/feedback_selected", act_state_topic))
    {
        throw_error_and_shutdown("/asl_gremlin/state_feedback/feedback_selected",
                                __LINE__);
    }

    vehicle_state_sub_ = nh.subscribe(act_state_topic,20,
                                      &GetSelectedFeedback::vehicle_state_callback,
                                      this);
    
    actual_state_ = new asl_gremlin_msgs::VehicleState();
}

GetSelectedFeedback::~GetSelectedFeedback()
{
    delete actual_state_;
}

void GetSelectedFeedback::vehicle_state_callback(const asl_gremlin_msgs::VehicleState::ConstPtr data)
{
    actual_state_->pose = data->pose;
    actual_state_->heading = data->heading;
}

asl_gremlin_msgs::VehicleState* 
GetSelectedFeedback::get_vehicle_state()
{
    return actual_state_;
}
