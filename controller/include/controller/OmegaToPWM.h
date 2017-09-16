#ifndef CONTROLLER__OMEGA_TO_PWM_H
#define CONTROLLER__OMEGA_TO_PWM_H

#include <vector>
#include <ros/ros.h>
#include <utility_pkg/custom_algorithms.h>

namespace controller{

class OmegaToPWM{
    std::vector<double> pwm_, omega_;

    public:
        OmegaToPWM(ros::NodeHandle&);
        double convert_omega_to_pwm(double);
};


} // end namespace


#endif
