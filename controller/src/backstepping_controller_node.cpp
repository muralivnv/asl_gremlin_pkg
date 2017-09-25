#include <controller/BackSteppingController_main.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <utility_pkg/error_util.h>


using namespace controller;
using namespace utility_pkg;
using namespace asl_gremlin_msgs;
using namespace asl_gremlin_pkg;

int main(int argc, char** argv)
{
    ros::init(argc, argv , "backstepping_controller"); 
    ros::NodeHandle ctrl_nh;
    
    std::string ref_traj_topic_name, act_state_topic;
    if (!ctrl_nh.getParam("/asl_gremlin/trajectory/publisher_topic",
                            ref_traj_topic_name) )
    {
        throw_error_and_shutdown("/asl_gremlin/trajectory/publisher_topic",
                                    __LINE__);
    }
    if (!ctrl_nh.getParam("/asl_gremlin/state_feedback/feedback_selected",
                            act_state_topic))
    {
        throw_error_and_shutdown("/asl_gremlin/state_feedback/feedback_selected",
                                    __LINE__);
    }

    SubscribeTopic<asl_gremlin_msgs::RefTraj> ref_traj(ctrl_nh, ref_traj_topic_name);
    SubscribeTopic<asl_gremlin_msgs::VehicleState> act_state(ctrl_nh, act_state_topic);
  
    
    ControllerBase<RefTraj, VehicleState>* controller = 
                            new BackSteppingController<RefTraj, VehicleState>(ctrl_nh);
    
    std::string ang_vel_topic;
    if (!ctrl_nh.getParam("/asl_gremlin/controller/cmd_angular_vel_topic",
                          ang_vel_topic))
    {
        utility_pkg::throw_error_and_shutdown("/asl_gremlin/controller/cmd_angular_vel_topic",
                                                __LINE__);
    }
    ros::Publisher ang_vel_cmd = ctrl_nh.advertise<MotorAngVel>
                                                    (ang_vel_topic, 20);
    ros::Rate loop_rate(10);
    ros::spinOnce();

    while(ros::ok())
    {
       controller->calculate_control_action(*(ref_traj.get_data()),
                                            *(act_state.get_data()));

       ang_vel_cmd.publish(*(controller->get_control_action()));

       ros::spinOnce();
       loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
