#include <ros/ros.h>
#include <asl_gremlin_msgs/MotorPwm.h>
#include <std_msgs/Bool.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_pkg/GetParam.h>
#include <std_msgs/Int16MultiArray.h>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_pwm_to_arduino");
    ros::NodeHandle pwm2ard_nh;

    std::string cmd_pwm_topic;

    cmd_pwm_topic = asl_gremlin_pkg::GetParam_with_shutdown<std::string>
                    (pwm2ard_nh,"/controller/cmd_pwm_topic", __LINE__);

    asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::MotorPwm> cmd_pwm(pwm2ard_nh, 
																		cmd_pwm_topic);

	asl_gremlin_pkg::SubscribeTopic<std_msgs::Bool> sim(pwm2ard_nh,ros::this_node::getNamespace()+"/start_sim");

    ros::Publisher pwm_pub = pwm2ard_nh.advertise<std_msgs::Int16MultiArray>
                                                        (ros::this_node::getNamespace()+"/arduino/cmd_pwm",20);

    double rate = 10.0;
    if (!pwm2ard_nh.getParam(ros::this_node::getNamespace()+"/sim/rate", rate))
    {
        ROS_WARN("Unable access parameter $robot_name/sim/rate, setting rate as 10Hz");
    }
    ros::Rate loop_rate(rate);
    
	std_msgs::Int16MultiArray pwm_arr;
	pwm_arr.data.clear();
	
	pwm_arr.data.push_back(0);
	pwm_arr.data.push_back(0);

	bool initiated = false;
	int pwm_left = 0, pwm_right = 0;

	while(ros::ok())
    {
    	if ( (sim.get_data())->data && !initiated )
    	{	initiated = true;  	}
    	
    	else if ( !(sim.get_data())->data && initiated )
    	{	initiated = false;  }
    	
    	if ( initiated )
    	{
    		pwm_left =  (cmd_pwm.get_data())->pwm_l;
    		pwm_right = (cmd_pwm.get_data())->pwm_r;
    	}
    	else
    	{	pwm_left = 0; pwm_right = 0;	}
    	
		pwm_arr.data[0] = -pwm_left;
		pwm_arr.data[1] = pwm_right;
        pwm_pub.publish(pwm_arr);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

}
