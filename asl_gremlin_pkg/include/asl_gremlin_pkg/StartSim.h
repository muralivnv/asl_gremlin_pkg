#ifndef _asl_gremlin_pkg_STARTSIM_H_
#define _asl_gremlin_pkg_STARTSIM_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace asl_gremlin_pkg{

class StartSim{
    ros::Subscriber start_sim_sub_;
    bool data_ = false;

    public:
        StartSim(ros::NodeHandle&);
        void start_sim_callback(const std_msgs::Bool::ConstPtr&);
        bool start();
};

} //end namespace{asl_gremlin_pkg}

#endif



