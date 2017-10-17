#include <ros/ros.h>                        // including "ros" libraries
#include <sensor_msgs/NavSatFix.h>          // including "NavSatFix" (for "/mavros/global_position/global" topic)
#include <geometry_msgs/PointStamped.h>     // PointStamped contains (X,Y,Z with timeStamp)
#include <std_msgs/Float64.h>               // including Float64 (for "/mavros/global/compass_hdg" topic)
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>                  // including Bool (for "/enco_reset" topic)
#include <state_feedback/Gps2xy.h>
#include <string>
#include <asl_gremlin_pkg/GetParam.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>

using namespace state_feedback;

// main program
int main(int argc, char **argv)
{
    Gps2xy gps2xy;

	ros::init(argc, argv, "gps2xy"); //initialising the node with name "gps2xy"
	ROS_INFO("Initialized:= %s",ros::this_node::getName().c_str());

	ros::NodeHandle nh; //creating nodehandle

    std::string gps_pub_topic = asl_gremlin_pkg::GetParam_with_shutdown<std::string>
                                (nh,"state_feedback/gps/pose_topic", __LINE__);

    // creating subscriber objects
	ros::Subscriber gps_sub = nh.subscribe("mavros/global_position/global", 1000, 
                                            &Gps2xy::gps_callback, &gps2xy); //subscribing to "GPS" topic

    ros::Subscriber ini_cond_set_pub = nh.subscribe(gps_pub_topic+"/set_ini_cond",10, 
                                            &Gps2xy::ini_cond_callback, &gps2xy);

    // creating publisher objects
	ros::Publisher local_pose_pub = nh.advertise< geometry_msgs::PointStamped >(gps_pub_topic, 100); //publishing the topic with topic name "/asl_gremlin1/gps2xy"
    asl_gremlin_pkg::SubscribeTopic <std_msgs::Bool> sim(nh, "start_sim");

	ros::Rate loop_rate(5); // running at 5Hz
	int count = 0;

    geometry_msgs::PointStamped local_pose; // Initialising global variables
    bool initiated = false;

    ros::spinOnce();


	while(ros::ok())
	{
        if ( (sim.get_data())->data && !initiated)
        {
            gps2xy.reset();
            count = 0;
            initiated = true;
        }
        if ( initiated && !(sim.get_data())->data)
        {
            initiated = true;
        }
        // first convert the current global to ecef co-ordinates
        gps2xy.geod2ecef();

        // now transform the ecef frame to local ENU frame
        gps2xy.ecef2enu();

		// publishing
		local_pose.header.seq = count; // current sequence
		local_pose.header.stamp = ros::Time::now(); // current time of publish
		local_pose.header.frame_id = "local_ENU"; // frame in which (X,Y) is specified

		local_pose.point.x = gps2xy.pos_ENU_x(); // assigning 'X'
		local_pose.point.y = gps2xy.pos_ENU_y(); // assigning 'Y'
		local_pose.point.z = gps2xy.pos_ENU_z(); // assigning 'Z'

		local_pose_pub.publish(local_pose);

		ros::spinOnce(); // to refresh all callbacks
		loop_rate.sleep();
        count++;
	}
}
