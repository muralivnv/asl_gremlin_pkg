/**
 * @brief TrajectoryBase header
 * @file TrajectoryBase.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _trajectory_generation_TRAJECTORYBASE_H_
#define _trajectory_generation_TRAJECTORYBASE_H_

#include <iostream>
#include <asl_gremlin_msgs/RefTraj.h>

class TrajectoryBase{

    public:
        virtual ~TrajectoryBase(){}
        virtual void update_start_time(double) = 0;
        virtual void set_ini_pose(double = 0.0, double = 0.0, double = 0.0) = 0;
        virtual void set_final_pose(double, double, double = 0.0) = 0;
        virtual void calc_coeff() = 0;
        virtual void generate_traj(double) = 0;
        virtual asl_gremlin_msgs::RefTraj* get_trajectory() = 0;
        virtual void set_current_pose_as_ini() = 0;
};

#endif 
