#include <trajectory_generation/trajectory_publisher.h>

void trajectory_generation::publish_trajectory(ros::Publisher& traj_pub, TrajectoryBase* trajectory)
{
    traj_pub.publish(*(trajectory->get_trajectory()));
}
