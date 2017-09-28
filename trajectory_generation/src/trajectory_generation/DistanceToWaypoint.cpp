#include <trajectory_generation/DistanceToWaypoint.h>

using namespace trajectory_generation;

DistanceToWaypoint::DistanceToWaypoint(ros::NodeHandle& nh)
{
    std::string feedback_selected_topic = 
                                    asl_gremlin_pkg::GetParam_with_shutdown<std::string>(nh, 
                                                                                        "/state_feedback/feedback_selected",
                                                                                        __LINE__);

    vehicle_state_ = new asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::VehicleState>
                                                        (nh, feedback_selected_topic);

    ros::spinOnce();
}

DistanceToWaypoint::~DistanceToWaypoint()
{
    delete vehicle_state_;
}

void DistanceToWaypoint::set_waypoint(double x, double y)
{
    x_wp_ = x; y_wp_ = y;
}

bool DistanceToWaypoint::is_reached_waypoint()
{
    double x_current = (vehicle_state_->get_data())->pose.point.x;
    double y_current = (vehicle_state_->get_data())->pose.point.y;

    return std::sqrt( std::pow(x_current - x_wp_,2) + 
                        (std::pow(y_current - y_wp_, 2))) < 
            0.8 ? true : false;
}
