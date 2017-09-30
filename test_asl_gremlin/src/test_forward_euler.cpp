#include <ros/ros.h>
#include <state_feedback/ForwardEuler.h>
#include <std_msgs/Float64.h>

#include <cmath>

using namespace state_feedback::numerical_diff;
using namespace state_feedback;

double diff_eq(double time,double state, void* nothing)
{
    return state;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv,"test_forward_euler");
    ros::NodeHandle enco2w_nh;

    ros::Rate loop_rate(5);
    double initial  = 1.0, integrated_states = 0.0;
    
    double t_initial = 0.0, t_final = 1;
    forward_euler_step_size = 0.0005;

    while (t_final < 6)
    {
        integrated_states = forwardEuler_integration(diff_eq,
                                                    initial,
                                                    t_initial, 
                                                    t_final);
        std::cout << integrated_states << '\n';
        initial = integrated_states;

        t_initial = t_final;
        t_final = t_initial + 1;
    }
}

