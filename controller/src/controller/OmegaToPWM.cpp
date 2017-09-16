#include <controller/OmegaToPWM.h>
#include <utility_pkg/custom_algorithms.h>

using namespace controller;
using namespace utility_pkg::custom_algorithms;

OmegaToPWM::OmegaToPWM(ros::NodeHandle& nh)
{
    if (!nh.getParam("/asl_gremlin/motor/omega_to_pwm/omega", omega_))
    {
        ROS_ERROR("Can't access parameter /asl_gremlin/motor/omega_to_pwm/omega");
        ros::shutdown();
    }

    if (!nh.getParam("/asl_gremlin/motor/omega_to_pwm/pwm",  pwm_))
    {
        ROS_ERROR("Can't access parameter /asl_gremlin/motor/omega_to_pwm/pwm");
        ros::shutdown();
    }

}

double OmegaToPWM::convert_omega_to_pwm(double omega_cmd)
{
   int idx = get_lower_index(omega_, omega_cmd);
   return linear_interpolate(pwm_,omega_, idx, omega_cmd);
}

