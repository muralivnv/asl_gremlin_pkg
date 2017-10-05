#include <controller/BackSteppingController.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_pkg/GetParam.h>
#include <std_msgs/Bool.h>

using namespace controller;
using namespace asl_gremlin_msgs;
using namespace asl_gremlin_pkg;

int main(int argc, char** argv)
{
    ros::init(argc, argv , "backstepping_controller"); 
    ros::NodeHandle ctrl_nh;
    
    std::string ref_traj_topic_name, act_state_topic;
    ref_traj_topic_name = GetParam_with_shutdown<std::string>
                            (ctrl_nh, "/trajectory/publisher_topic", __LINE__);

    act_state_topic = GetParam_with_shutdown<std::string>
                        (ctrl_nh,"/state_feedback/feedback_selected", __LINE__); 

    SubscribeTopic<asl_gremlin_msgs::RefTraj> ref_traj(ctrl_nh, ref_traj_topic_name);
    SubscribeTopic<asl_gremlin_msgs::VehicleState> act_state(ctrl_nh, act_state_topic);
  
    
    ControllerBase<RefTraj, VehicleState>* controller = 
                            new BackSteppingController<RefTraj, VehicleState>(ctrl_nh);
    
    std::string ang_vel_topic;
    ang_vel_topic = GetParam_with_shutdown<std::string>
                        (ctrl_nh, "/controller/cmd_angular_vel_topic",__LINE__);

    asl_gremlin_pkg::SubscribeTopic<std_msgs::Bool> sim(ctrl_nh, ros::this_node::getNamespace()+"/start_sim"); 

    ros::Publisher ang_vel_cmd = ctrl_nh.advertise<MotorAngVel>
                                                    (ang_vel_topic, 20);
 
    int rate = 10;
    if (!ctrl_nh.getParam(ros::this_node::getNamespace()+"/sim/rate", rate))
    {
        ROS_WARN("Unable access parameter $robot_name/sim/rate, setting rate as 10Hz");
    }
    ros::Rate loop_rate(rate);
    ros::spinOnce();

    ROS_INFO("Initialized /backstepping_controller_node");
    while(ros::ok())
    {
        if ( (sim.get_data())->data )
        {
            ROS_INFO_ONCE("Started generating control commands");
            controller->calculate_control_action(*(ref_traj.get_data()),
                                                *(act_state.get_data()));
        }
        else
        { controller->reset(); }

       ang_vel_cmd.publish(*(controller->get_control_action()));

       ros::spinOnce();
       loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}
