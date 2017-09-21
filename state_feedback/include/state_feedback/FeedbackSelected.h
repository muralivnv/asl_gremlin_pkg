#ifndef _state_feedback_FEEDBACKSELECTED_H_
#define _state_feedback_FEEDBACKSELECTED_H_

#include <dynamic_reconfigure/server.h>
#include <state_feedback/feedbackSelectConfig.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <utility_pkg/error_util.h>
#include <array>
#include <asl_gremlin_msgs/VehicleState.h>
#include "GetCompassHdg.h"

enum Feedback{
    gps_compass = 0,
    encoder_compass = 1,
    pure_gps = 2
};


using pose_type = asl_gremlin_msgs::VehicleState;

namespace state_feedback{

template<int N>
class FeedbackSelected{

    pose_type* pose_;

    int feedback = Feedback::gps_compass;

    dynamic_reconfigure::Server<feedbackSelectConfig> dr_feedback_srv_;
    dynamic_reconfigure::Server<feedbackSelectConfig>::CallbackType fun_;

    ros::Publisher feedback_pub_;
    ros::Subscriber gps_pose_sub_, enco_pose_sub_;

    GetCompassHdg* compass_hdg_;

    public:
        FeedbackSelected(ros::NodeHandle&);
        ~FeedbackSelected();

        void dynamic_reconfigure_feedback_callback(feedbackSelectConfig& , uint32_t);
        void gps_to_pose_callback(const geometry_msgs::PointStamped::ConstPtr );
        void encoder_to_pose_callback(const geometry_msgs::PointStamped::ConstPtr );
        void publish();
};

template<int N>
FeedbackSelected<N>::FeedbackSelected(ros::NodeHandle& nh) : pose_(new pose_type[N])
{                
    fun_ = boost::bind(&FeedbackSelected::dynamic_reconfigure_feedback_callback,
                        this, _1, _2);

    dr_feedback_srv_.setCallback(fun_);

    std::string feedback_topic, enco_pose_topic, gps_pose_topic;

    if (!nh.getParam("/asl_gremlin/state_feedback/feedback_selected", feedback_topic))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/state_feedback/feedback_selected",
                                                __LINE__);
    }
    if (!nh.getParam("/asl_gremlin/state_feedback/encoder/pose_topic", enco_pose_topic))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/state_feedback/encoder/pose_topic",
                                                __LINE__);
    }

    if (!nh.getParam("/asl_gremlin/state_feedback/gps/pose_topic", gps_pose_topic))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/state_feedback/gps/pose_topic",
                                                __LINE__);
    }

    
    feedback_pub_ = nh.advertise<pose_type>(feedback_topic, 10);

    gps_pose_sub_ = nh.subscribe(gps_pose_topic, 10, 
                                    &FeedbackSelected<N>::gps_to_pose_callback,this);

    enco_pose_sub_ = nh.subscribe(enco_pose_topic, 10, 
                                    &FeedbackSelected<N>::encoder_to_pose_callback,this);

    compass_hdg_ = new GetCompassHdg(nh);

    ros::spinOnce();
}

template<int N>
FeedbackSelected<N>::~FeedbackSelected()
{
    delete[] pose_;
    delete compass_hdg_;
}

template<int N>
void FeedbackSelected<N>::dynamic_reconfigure_feedback_callback(feedbackSelectConfig& config, uint32_t level)
{
    feedback = config.feedback;
}

template<int N>
void FeedbackSelected<N>::gps_to_pose_callback(const geometry_msgs::PointStamped::ConstPtr data)
{
    double theta_enu = compass_hdg_->data_ENU();

    pose_[0].pose.point = data->point;
    pose_[0].pose.header = data->header;
    pose_[0].heading = theta_enu;

    pose_[1].pose.point.z = data->point.z;
    pose_[1].heading = theta_enu;
    
    double theta_gps = std::atan2(data->point.y - pose_[2].pose.point.y,
                                    data->point.x - pose_[2].pose.point.x);

    pose_[2].pose.point = data->point;
    pose_[2].pose.header = data->header;
    pose_[2].heading = theta_gps;
}


template<int N>
void FeedbackSelected<N>::encoder_to_pose_callback(const geometry_msgs::PointStamped::ConstPtr data)
{
    pose_[1].pose.header = data->header;
    pose_[1].pose.point.x = data->point.x;
    pose_[1].pose.point.y = data->point.y;
}


template<int N>
void FeedbackSelected<N>::publish()
{
    feedback_pub_.publish(pose_[feedback]);
}

} // end namespace { state_feedback}
#endif
