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

namespace controller{

double delta_theta(double theta_act, double theta_des)
{
	double delta1 = theta_act - theta_des;
	double delta2 = -((M_PI - theta_act) - (-M_PI - theta_des));
	double delta3 = ((M_PI - theta_des) - (-M_PI - theta_act));

	double delta = 0.0;

	if (std::fabs(delta1) < std::fabs(delta2))
	{	delta = delta1;	}
	else
	{   delta = delta2; }

	if (std::fabs(delta) > std::fabs(delta3))
	{   delta = delta3;	}
	
	return delta;
}

} // end namespace { controller }

#endif
