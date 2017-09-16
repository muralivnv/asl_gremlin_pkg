#ifndef UTILITY_PKG__UTILITIES_H
#define UTILITY_PKG__UTILITIES_H

namespace utility_pkg{

// function which converts the compass angle (0 to 360) NED to
// (-180 to 180) ENU frame
double compass_angle_to_polar_angle(double theta_NED)
{
	double theta_ENU = 0.0;
    // if the compass angle is in 1st and 4th quadrants of ENU
	if (theta_NED >= 0 && theta_NED <= 180)
	{
		theta_ENU = 90 - theta_NED;
	}
    // else if compass angle is in 2nd quadrant of ENU
	else if (theta_NED > 270 && theta_NED <= 360)
	{
		theta_ENU = (360 - theta_NED) + 90;
	}
    // else if compass angle is in 3rd quadrant of ENU
	else if (theta_NED > 180 && theta_NED <= 270)
	{
		theta_ENU = -((theta_NED + 90) - 180);
	}
	return theta_ENU;
}


} // end namepace {utility_pkg}
#endif
