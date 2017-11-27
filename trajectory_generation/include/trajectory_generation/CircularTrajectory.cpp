/**
 * @brief Generate CircularTrajectory header and definitions
 * @file CircularTrajectory.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _trajectory_generation_CIRCULARTRAJECTORY_H_
#define _trajectory_generation_CIRCULARTRAJECTORY_H_

#include <iostream>
#include <ros/ros.h>
#include <array>
#include <asl_gremlin_msgs/RefTraj.h>

template<typename ParamType>
class CircularTrajectory{
	ParamType* params_;
	
	int turn_dir = 0;
	double circle_start_angle = 0.0, circle_end_angle = 0.0;
}