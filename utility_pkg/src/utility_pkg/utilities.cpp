/**
 * @brief system utilities definitions
 * @file utilities.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#include <utility_pkg/utilities.h>

// function which converts the compass angle (0 to 360) NED to
// (-180 to 180) ENU frame
double utility_pkg::compass_angle_to_polar_angle(double theta_NED)
{
	double theta_ENU = 0.0;
    // if compass angle is in 2nd quadrant of ENU frame
    if (theta_NED > 270 && theta_NED <= 360)
	{ theta_ENU = (360 - theta_NED) + 90; }

	// otherwise
	else
	{ theta_ENU = 90 - theta_NED; }

	return theta_ENU;
}

double utility_pkg::wrapTo2Pi(double theta)
{
	if (theta < 0)
	{ return 2*M_PI + theta; }
    else if (theta > 0)
    { 
        theta = std::fmod(theta, 2*M_PI);
        if (theta == 0.0)
        { return 2*M_PI; }
    }

	return theta; 
}
