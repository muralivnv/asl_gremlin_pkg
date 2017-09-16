#ifndef TRAJECTORY_GENERATION__TRAJECTORY_PUBLISHER_HPP
#define TRAJECTORY_GENERATION__TRAJECTORY_PUBLISHER_HPP

#include "TrajectoryBase.h"
#include <ros/ros.h>

namespace trajectory_generation{

void publish_trajectory(ros::Publisher& , TrajectoryBase* );
}

#endif
