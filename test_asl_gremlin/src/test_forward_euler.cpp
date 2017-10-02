#include <ros/ros.h>
#include <state_feedback/ForwardEuler.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <array>

using namespace state_feedback::numerical_diff;
using namespace state_feedback;

struct paramType{
    double wl = 11.2;
    double wr = 11.2;
    double r  = 0.06858;
    double b  = 0.3353; 
};


std::array<double,3> diff_eq(double time,std::array<double,3> state, paramType* param)
{
    double x_dot = (param->r)*0.5*(param->wr+param->wl)*cos(state[2]);
    double y_dot = (param->r)*0.5*(param->wr+param->wl)*sin(state[2]);

    double theta_dot = (param->r/param->b)*(param->wr - param->wl);

    return {{x_dot, y_dot, theta_dot}};
}


int main(int argc, char** argv)
{
    ros::init(argc, argv,"test_forward_euler");
    ros::NodeHandle enco2w_nh;

    ros::Rate loop_rate(5);
    std::array<double, 3> initial{{0,0, M_PI/4}}, integrated_states{0,0,0};
    
    double t_initial = 0.0, t_final = 0.1;
    paramType param;
    forward_euler_step_size = 0.1;

    while (t_final <= 10)
    {
        integrated_states = forwardEuler_integration(diff_eq,
                                                    initial,
                                                    t_initial, 
                                                    t_final,&param);
        std::cout << integrated_states[0] << "   " << integrated_states[1] <<'\n';
        initial[0] = integrated_states[0];
        initial[1] = integrated_states[1];

        t_initial = t_final;
        t_final = t_initial + 0.1;
    }
}

