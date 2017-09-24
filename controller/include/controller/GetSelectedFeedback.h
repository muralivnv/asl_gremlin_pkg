#ifndef _controller_GETSELECTEDFEEDBACK_H_
#define _controller_GETSELECTEDFEEDBACK_H_

#include <ros/ros.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <utility_pkg/error_util.h>
#include <string>

namespace controller{

class GetSelectedFeedback{
    asl_gremlin_msgs::VehicleState* actual_state_;
    ros::Subscriber vehicle_state_sub_;

    public:
        GetSelectedFeedback(ros::NodeHandle&);
        ~GetSelectedFeedback();

        void vehicle_state_callback(const asl_gremlin_msgs::VehicleState::ConstPtr);

        asl_gremlin_msgs::VehicleState* get_vehicle_state();
};


} //end namespace {controller}
#endif
