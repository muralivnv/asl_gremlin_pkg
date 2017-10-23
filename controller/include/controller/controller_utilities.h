/**
 * @brief Controller-Utilities header
 * @file controller_utilities.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _controller_CONTROLLERUTLITIES_H_
#define _controller_CONTROLLERUTILITIES_H_

#include <iostream>
#include <cmath>


static constexpr double deg2rad = M_PI/180.0;
static constexpr double rad2deg = 180.0/M_PI;
static constexpr double quadrant_ratio = 4/360.0;

#define FIRST_QUADRANT 1
#define SECOND_QUADRANT 2
#define THIRD_QUADRANT 3
#define FOURTH_QUADRANT 4

namespace controller{

inline int get_angle_quadrant(double angle)
{
    angle = angle*rad2deg;

    if (angle < 0.0)
    { angle = angle + 360.0; }

   double quadrant = (angle*quadrant_ratio);

   return quadrant + (fmod(quadrant,1.0) == 0 ? 0 : 1);
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
