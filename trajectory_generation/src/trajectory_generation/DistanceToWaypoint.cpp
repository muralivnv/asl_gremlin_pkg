#include <trajectory_generation/DistanceToWaypoint.h>

using namespace trajectory_generation;

DistanceToWaypoint::DistanceToWaypoint(ros::NodeHandle& nh)
{
    std::string feedback_selected_topic;
    if (!nh.getParam("/asl_gremlin/state_feedback/feedback_selected", 
                        feedback_selected_topic))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/state_feedback/feedback_selected",__LINE__);
    }

    vehicle_state_sub_ = nh.subscribe(feedback_selected_topic, 10
                                     &DistanceToWaypoint::vehicle_state_callback,
                                     this);
    ros::spinOnce();
}


void DistanceToWaypoint::vehicle_state_callback(const asl_gremlin_msgs::VehicleState::ConstPtr data)
{
    x_current_ = data->pose.point.x;
    y_current_ = data->pose.point.y;
}

void DistanceToWaypoint::set_waypoint(double x, double y)
{
    x_wp_ = x; y_wp_ = y;
}

bool DistanceToWaypoint::is_reached_waypoint()
{
    return std::sqrt( std::pow(x_current_ - x_wp_,2) + 
                        (std::pow(y_current_ - y_wp_, 2))) < 
            0.8 ? true : false;
}
