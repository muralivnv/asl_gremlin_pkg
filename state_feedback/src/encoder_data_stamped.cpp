#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_pkg/GetParam.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"encoder_stamped_node");
	ros::NodeHandle nh;

    asl_gremlin_pkg::SubscribeTopic<std_msgs::Int32>    left_encoder_data(nh, "/enco_L", 100),
                                                        right_encoder_data(nh, "/enco_R", 100),
                                                        reset_encoder(nh, "/enco_reset", 10);

    std::string left_encoder_stamped_topic, right_encoder_stamped_topic;
    left_encoder_stamped_topic = asl_gremlin_pkg::GetParam_with_shutdown<std::string>(nh, 
                                                                        "/state_feedback/encoder/left_timeStamped_topic",
                                                                        __LINE__);

    right_encoder_stamped_topic = asl_gremlin_pkg::GetParam_with_shutdown<std::string>(nh, 
                                                                        "/state_feedback/encoder/right_timeStamped_topic",
                                                                        __LINE__);

	ros::Publisher left_encoder_data_stamped  = nh.advertise < std_msgs::Float64MultiArray >(left_encoder_stamped_topic, 100);
	ros::Publisher right_encoder_data_stamped = nh.advertise < std_msgs::Float64MultiArray >(right_encoder_stamped_topic,100);

	std_msgs::Float64MultiArray msg_right;
	std_msgs::Float64MultiArray msg_left;
    double count = 0.0;

	msg_right.data.clear();
	msg_left.data.clear();

	ros::Rate loop_rate(250);

	while(ros::ok())
	{
		msg_right.data.clear();
		msg_left.data.clear();

		msg_right.data.push_back((right_encoder_data.get_data())->data);
		msg_right.data.push_back(ros::Time::now().toSec());
		msg_right.data.push_back(count);

		msg_left.data.push_back((left_encoder_data.get_data())->data);
		msg_left.data.push_back(ros::Time::now().toSec());
		msg_left.data.push_back(count);

		left_encoder_data_stamped.publish(msg_left);
		right_encoder_data_stamped.publish(msg_right);

		loop_rate.sleep();
		ros::spinOnce();

		++count;

        if ((reset_encoder.get_data())->data)
        { count = 0.0; }
	}
}
