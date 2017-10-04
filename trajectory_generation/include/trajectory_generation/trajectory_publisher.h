#ifndef _trajectory_generation_TRAJECTORYPUBLISHER_H_
#define _trajectory_generation_TRAJECTORYPUBLISHER_H_

#include "TrajectoryBase.h"
#include <ros/ros.h>

namespace trajectory_generation{

void publish_trajectory(ros::Publisher& , TrajectoryBase* );
}

#endif
