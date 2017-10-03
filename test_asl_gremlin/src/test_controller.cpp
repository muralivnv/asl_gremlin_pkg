#include <controller/BackSteppingController_main.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <asl_gremlin_pkg/GetParam.h>
#include <fstream>

using namespace controller;
using namespace asl_gremlin_msgs;
using namespace asl_gremlin_pkg;

int main(int argc, char** argv)
{
    ros::init(argc, argv , "test_controller"); 
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

    ros::Publisher ang_vel_cmd = ctrl_nh.advertise<MotorAngVel>
                                                    (ang_vel_topic, 20);
    ros::Rate loop_rate(10);
    ros::spinOnce();

    std::ofstream ang_vel_log("/home/vnv/asl_gremlin1/src/test_asl_gremlin/src/cpp_output_data/cmd_ang_vel.dat");
    
    assert(ang_vel_log.is_open());
    ang_vel_log << "#wl" << "    " << "#wr\n";
    while(ros::ok())
    {
       controller->calculate_control_action(*(ref_traj.get_data()),
                                            *(act_state.get_data()));

       ang_vel_cmd.publish(*(controller->get_control_action()));
        
       ang_vel_log << (controller->get_control_action())->wl << "    "<< (controller->get_control_action())->wr <<'\n';
       //std::cout << (controller->get_control_action())->wl << "    "<<(controller->get_control_action())->wr <<'\n';

       ros::spinOnce();
       loop_rate.sleep();
    }

    ang_vel_log.close();
    return EXIT_SUCCESS;
}
