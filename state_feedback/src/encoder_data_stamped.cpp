/**
 * @brief Timestamp encoder data from arduino node
 * @file encoder_data_stamped.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"encoder_stamped_node");

	ros::NodeHandle nh;

    asl_gremlin_pkg::SubscribeTopic<std_msgs::Int32>    left_encoder_data(nh, "arduino/left_encoder", 250),
                                                        right_encoder_data(nh, "arduino/right_encoder", 250),
                                                        reset_encoder(nh, "reset_encoders", 10);

    std::string left_encoder_stamped_topic, right_encoder_stamped_topic;
    if(!nh.getParam("state_feedback/encoder/left_timeStamped_topic", left_encoder_stamped_topic))
    { left_encoder_stamped_topic = "state_feedback/encoder_left_timeStamped"; }

    if(!nh.getParam("state_feedback/encoder/right_timeStamped_topic", right_encoder_stamped_topic))
    { right_encoder_stamped_topic = "state_feedback/encoder_right_timeStamped"; }


	ros::Publisher left_encoder_data_stamped  = nh.advertise < std_msgs::Float64MultiArray >(left_encoder_stamped_topic, 250);
	ros::Publisher right_encoder_data_stamped = nh.advertise < std_msgs::Float64MultiArray >(right_encoder_stamped_topic,250);

	std_msgs::Float64MultiArray msg_right;
	std_msgs::Float64MultiArray msg_left;
    double count = 0.0;

	msg_right.data.clear();
	msg_left.data.clear();

    msg_right.data.push_back(0.0);
    msg_right.data.push_back(0.0);
    msg_right.data.push_back(0.0);

    msg_left.data.push_back(0.0);
    msg_left.data.push_back(0.0);
    msg_left.data.push_back(0.0);

	ros::Rate loop_rate(250);
	

	ROS_INFO("\033[1;32mInitialized\033[0;m:= %s",ros::this_node::getName().c_str());
	while(ros::ok())
	{
		msg_right.data[0] = ((right_encoder_data.get_data())->data);
		msg_right.data[1] = (ros::Time::now().toSec());
		msg_right.data[2] = (count);

		msg_left.data[0] = ((left_encoder_data.get_data())->data);
		msg_left.data[1] = (ros::Time::now().toSec());
		msg_left.data[2] = (count);

		left_encoder_data_stamped.publish(msg_left);
		right_encoder_data_stamped.publish(msg_right);

		loop_rate.sleep();
		ros::spinOnce();

		++count;

        if ((reset_encoder.get_data())->data)
        {
            ROS_INFO("\033[0;33mRestted encoders\033[0;m");
            count = 0.0;
        }
	}
}
