#ifndef _controller_GETREFTRAJ_H_
#define _controller_GETREFTRAJ_H_

#include <ros/ros.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <utility_pkg/error_util.h>
#include <string>

namespace controller{

class GetRefTraj{
        asl_gremlin_msgs::RefTraj* ref_trajectory_;
        ros::Subscriber ref_traj_sub_;
    public:
        GetRefTraj(ros::NodeHandle&);
        ~GetRefTraj();
        void ref_traj_callback(const asl_gremlin_msgs::RefTraj::ConstPtr);
        asl_gremlin_msgs::RefTraj* get_ref_traj();
};

} // end namespace {controller}

#endif
