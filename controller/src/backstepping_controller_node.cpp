#include <controller/BackSteppingController_main.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int16MultiArray.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <controller/GetRefTraj.h>
#include <controller/GetSelectedFeedback.h>
#include <utility_pkg/error_util.h>


using namespace controller;
using namespace asl_gremlin_msgs;

int main(int argc, char** argv)
{
    ros::init(argc, argv , "backstepping_controller"); 
    ros::NodeHandle ctrl_nh;
    
    GetRefTraj ref_traj(ctrl_nh);
    GetSelectedFeedback act_state(ctrl_nh);
    
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
       controller->calculate_control_action(*(ref_traj.get_ref_traj()),
                                            *(act_state.get_vehicle_state()));

       ang_vel_cmd.publish(*(controller->get_control_action()));

       ros::spinOnce();
       loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
