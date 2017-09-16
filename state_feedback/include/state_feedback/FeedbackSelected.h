#ifndef _state_feedback_FEEDBACKSELECTED_H_
#define _state_feedback_FEEDBACKSELECTED_H_

#include <dynamic_reconfigure/server.h>
#include <state_feedback/feedbackSelectConfig.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <utility_pkg/error_util.h>
#include <array>

enum feedback{
    gps_compass = 0,
    encoder_compass=1,
    pure_gps = 2
};


using state_feedback::vehicle_state = pose_type;

namespace state_feedback{

template<int N>
class FeedbackSelected{

    pose_type* pose_;
    int feedback = feedback::gps_compass;

    dynamic_reconfigure::Server<feedbackSelectConfig> dr_feedback_srv_;
    dynamic_reconfigure::Server<feedbackSelectConfig>::CallbackType fun_;

    public:
        FeedbackSelected(ros::NodeHandle&) : 
            pose_(new pose_type[N]){
                
            fun_ = boost::bind(&FeedbackSelected::dynamic_reconfigure_feedback_callback,
                                this, _1, _2);

            dr_feedback_srv_.setCallback(fun_);
            }

        ~FeedbackSelected();

        void dynamic_reconfigure_feedback_callback(feedbackSelectConfig& , uint32_t);
        void gps_to_pose_callback(const geometry_msgs::PointStamped::ConstPtr );
        void encoder_to_pose_callback(const geometry_msgs::PointStamped::ConstPtr );

        void publish();
};


template<int N>
FeedbackSelected<N>::~FeedbackSelected()
{
    delete[] pose_;
}

template<int N>
void FeedbackSelected<N>::dynamic_reconfigure_feedback_callback(feedbackSelectConfig& config, uint32_t level)
{
    feedback = config.feedback;
}

template<int N>
void FeedbackSelected<N>::gps_to_pose_callback(const geometry_msgs::PointStamped::ConstPtr data)
{
    pose_[0].x = data->point.x;
    pose_[0].y = data->point.y;
    pose_[0].z = data->point.z;
    pose_[0].theta = 
}


} // end namespace { state_feedback}
#endif
