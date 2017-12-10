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
	{ 
        theta =  2*M_PI + theta;
        if (theta < 0)
        { theta = wrapTo2Pi(theta); }
        return theta;
    }
    else if (theta > 2*M_PI)
    { 
        theta = std::fmod(theta, 2*M_PI);
        if (theta == 0.0)
        { return 2*M_PI; }
    }

	return theta; 
}

void utility_pkg::stop_rover(const std::string& rover_name)
{
    ROS_INFO("\033[1;33mReached all waypoints, stopping rover\033[0;m");

    std::string topic_name = rover_name +"/start_sim";

    std::size_t end_idx = topic_name.find_first_not_of('/');
    topic_name = '/' + topic_name.substr(end_idx);
    
    std::string cmd = "rostopic pub --once "+ topic_name + " std_msgs/Bool \"data: false\"";

    auto res = std::system(cmd.c_str());
}

std::string utility_pkg::exec_cmd(const std::string& cmd)
{
    std::array<char, 256> buffer;
    std::string cmd_output;

    FILE* pipe= popen(cmd.c_str(), "r");

    if (!pipe)
    { throw std::runtime_error("popen() failed"); }

    while (!feof(pipe))
    {
        if (fgets(buffer.data(), 256, pipe) != nullptr)
        { cmd_output += buffer.data(); }
    }
    pclose(pipe);
    return cmd_output;
}

std::string utility_pkg::get_robot_name(char** argv)
{
    std::string exec_path(argv[0]);
    std::string robot_name(exec_cmd("source "+ exec_path+"src/bash_scripts/this_robot_name.sh"));
    robot_name = exec_cmd("echo $ROBOT_NAME");
    return robot_name;
}