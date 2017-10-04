#ifndef _controller_CONTROLLERUTLITIES_H_
#define _controller_CONTROLLERUTILITIES_H_

#include <iostream>
#include <cmath>


#define FIRST_QUADRANT 1
#define SECOND_QUADRANT 2
#define THIRD_QUADRANT 3
#define FOURTH_QUADRANT 4


namespace controller{

int get_angle_quadrant(double angle)
{
    double sin_of_angle = std::sin(angle);
    double cos_of_angle = std::cos(angle);

    if (sin_of_angle >= 0 && cos_of_angle >= 0)
    { return FIRST_QUADRANT; }

    else if (sin_of_angle > 0 && cos_of_angle < 0)
    { return SECOND_QUADRANT; }

    else if (sin_of_angle <= 0 && cos_of_angle <= 0)
    { return THIRD_QUADRANT; }

    else if (sin_of_angle < 0 && cos_of_angle > 0)
    { return FOURTH_QUADRANT; }

    else
    { return FIRST_QUADRANT; }
}


double delta_theta(double theta_act, double theta_ref)
{
    int rover_quadrant   = get_angle_quadrant(theta_act);
    int ref_hdg_quadrant = get_angle_quadrant(theta_ref);
    
    double delta_theta = 0.0;

    switch(rover_quadrant)
    {
        case(SECOND_QUADRANT):
            switch(ref_hdg_quadrant)
            {
                case(FIRST_QUADRANT): case(SECOND_QUADRANT):
                    delta_theta = (theta_act - theta_ref);
                    break;

                case(THIRD_QUADRANT):
                    delta_theta = -((M_PI-theta_act) - (-M_PI-theta_ref));
                    break;

                case(FOURTH_QUADRANT):
                    if(fabs(theta_act-theta_ref) <= fabs(-(M_PI-theta_act-(-M_PI-theta_ref))))
                    { delta_theta = (theta_act - theta_ref); }
                    else
                    { delta_theta = -(M_PI-theta_act-(-M_PI-theta_ref)); }
                    break;
            }
            break;

        case(THIRD_QUADRANT):
            switch(ref_hdg_quadrant)
            {
                case(THIRD_QUADRANT): case(FOURTH_QUADRANT):
                    delta_theta = (theta_act - theta_ref);
                    break;

                case(SECOND_QUADRANT):
                    delta_theta = ((M_PI-theta_ref) - (-M_PI-theta_act));
                    break;

                case(FIRST_QUADRANT):
                    if(fabs((M_PI-theta_ref) - (-M_PI-theta_act)) <= fabs(theta_act-theta_ref))
                    { delta_theta = (M_PI-theta_ref) - (-M_PI-theta_act); }
                    else
                    { delta_theta = (theta_act - theta_ref); }
                    break;
            }
            break;

      case(FIRST_QUADRANT):
            switch(ref_hdg_quadrant)
            {
                case(FIRST_QUADRANT): case(SECOND_QUADRANT): case(FOURTH_QUADRANT):
                    delta_theta = (theta_act - theta_ref);
                    break;

                 case(THIRD_QUADRANT):
                    if(fabs((M_PI-theta_act) - (-M_PI-theta_ref)) <= fabs(theta_act-theta_ref))
                    { delta_theta = -((M_PI-theta_act) - (-M_PI-theta_ref)); }
                    else
                    { delta_theta = (theta_act - theta_ref); }
                    break;
            }
            break;

      case(FOURTH_QUADRANT):
            switch(ref_hdg_quadrant)
            {
                case(FIRST_QUADRANT): case(THIRD_QUADRANT): case(FOURTH_QUADRANT):
                    delta_theta = (theta_act - theta_ref);
                    break;

                case(SECOND_QUADRANT):
                  if(fabs(theta_act-theta_ref) <= fabs(M_PI-theta_ref-(-M_PI-theta_act)))
                  { delta_theta = theta_act - theta_ref; }
                  else
                  { delta_theta = ((M_PI - theta_ref) - (-M_PI-theta_act)); }
                  break;
            }
            break;
    }
    return delta_theta;
}

}

#endif
