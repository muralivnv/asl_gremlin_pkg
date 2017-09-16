#ifndef TRAJECTORY_GENERATION_TRAJECTORY_BASE_H
#define TRAJECTORY_GENERATION_TRAJECTORY_BASE_H

#include <iostream>
#include <trajectory_generation/ref_traj.h>

class TrajectoryBase{

    public:
        virtual ~TrajectoryBase(){}
	virtual void update_start_time(double) = 0;
        virtual void set_ini_pose(double = 0.0, double = 0.0, double = 0.0) = 0;
        virtual void set_final_pose(double, double, double = 0.0) = 0;
        virtual void calc_coeff() = 0;
        virtual void generate_traj(double) = 0;
        virtual trajectory_generation::ref_traj* get_trajectory() = 0;
	virtual void set_current_traj_value_to_ini() = 0;
};

#endif 
