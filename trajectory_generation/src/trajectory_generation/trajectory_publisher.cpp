/**
 * @brief trajectory_publisher definitions
 * @file trajectory_publisher.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#include <trajectory_generation/trajectory_publisher.h>

void trajectory_generation::publish_trajectory(ros::Publisher& traj_pub, TrajectoryBase* trajectory)
{ traj_pub.publish(*(trajectory->get_trajectory())); }
