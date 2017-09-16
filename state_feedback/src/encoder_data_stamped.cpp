#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

float enco_R_data = 0.0, enco_L_data = 0.0, count = 0.0;

void enco_R_callback(const std_msgs::Int32::ConstPtr& enco_R_msg)
{
	enco_R_data = static_cast<float>(enco_R_msg->data);
}

void enco_L_callback(const std_msgs::Int32::ConstPtr& enco_L_msg)
{
	enco_L_data = static_cast<float>(enco_L_msg->data);
}

void enco_reset_callback(const std_msgs::Bool::ConstPtr& enco_reset_msg)
{
	if (enco_reset_msg->data == true)
	{
		count = 0.0;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"encoder_stamped_node");
	ros::NodeHandle nh;

	ros::Subscriber enco_L_sub = nh.subscribe<std_msgs::Int32>("/enco_L",100,enco_L_callback);
	ros::Subscriber enco_R_sub = nh.subscribe<std_msgs::Int32>("/enco_R",100,enco_R_callback);
	ros::Subscriber enco_reset_sub = nh.subscribe<std_msgs::Bool>("/enco_reset",20,enco_reset_callback);

    std::string left_encoder_stamped_topic, right_encoder_stamped_topic;
    if (!nh.getParam("/asl_gremlin/state_feedback/encoder/left_timeStamped_topic", left_encoder_stamped_topic))
    {
        ROS_ERROR("can't access parameter: /asl_gremlin/state_feedback/encoder/left_timeStamped_topic");
        ros::shutdown();
    }

    if (!nh.getParam("/asl_gremlin/state_feedback/encoder/right_timeStamped_topic", right_encoder_stamped_topic))
    {
        ROS_ERROR("can't access parameter: /asl_gremlin/state_feedback/encoder/right_timeStamped_topic");
        ros::shutdown();
    }

	ros::Publisher enco_L_stamp = nh.advertise<std_msgs::Float64MultiArray>(left_encoder_stamped_topic,100);
	ros::Publisher enco_R_stamp = nh.advertise<std_msgs::Float64MultiArray>(right_encoder_stamped_topic,100);

	std_msgs::Float64MultiArray msg_R;
	std_msgs::Float64MultiArray msg_L;

	msg_R.data.clear();
	msg_L.data.clear();

	ros::Rate loop_rate(250);

	while(ros::ok())
	{
		msg_R.data.clear();
		msg_L.data.clear();

		msg_R.data.push_back(enco_R_data);
		msg_R.data.push_back(ros::Time::now().toSec());
		msg_R.data.push_back(count);

		msg_L.data.push_back(enco_L_data);
		msg_L.data.push_back(ros::Time::now().toSec());
		msg_L.data.push_back(count);

		enco_L_stamp.publish(msg_L);
		enco_R_stamp.publish(msg_R);

		loop_rate.sleep();
		ros::spinOnce();
		count = count + 1.0;
	}
}
