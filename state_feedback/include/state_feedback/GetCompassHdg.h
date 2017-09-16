#ifndef STATE_FEEDBACK__GET_COMPASS_HDG_H
#define STATE_FEEDBACK__GET_COMPASS_HDG_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <utility_pkg/utilities.h>

class GetCompassHdg{

    double compass_hdg_ = 0.0;
    ros::Subscriber hdg_subscribe_;
    
    public:
        GetCompassHdg(ros::NodeHandle& nh){
            hdg_subscribe_ = nh.subscribe("/mavros/global_position/compass_hdg",1,
                                          &GetCompassHdg::hdg_callback, this);
        }
        
        void hdg_callback(const std_msgs::Float64::ConstPtr);
        
        double data();
        double data_ENU();
};


#endif
