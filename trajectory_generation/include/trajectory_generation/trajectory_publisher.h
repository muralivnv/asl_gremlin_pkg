/**
 * @brief Publish reference trajectory header
 * @file trajectory_publisher.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _trajectory_generation_TRAJECTORYPUBLISHER_H_
#define _trajectory_generation_TRAJECTORYPUBLISHER_H_

#include "TrajectoryBase.h"
#include <ros/ros.h>

namespace trajectory_generation{

void publish_trajectory(ros::Publisher& , const std::unique_ptr<TrajectoryBase>& );
}

#endif
