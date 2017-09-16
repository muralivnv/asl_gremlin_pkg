#ifndef ASL_GREMLIN1_TRAJ_GEN_HPP
#define ASL_GREMLIN1_TRAJ_GEN_HPP

#include <iostream>
#include <array>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation/ref_traj.h>
#include <dynamic_reconfigure/server.h>
#include <trajectory_generation/waypointSetConfig.h>
#include <utility_pkg/str_manip.h>


class TrajGen{
    // array to store polynomial coefficients
    std::array<double, 6> X_coeff;
    std::array<double, 6> Y_coeff;

    // trajectory initializing time
    double init_time = 0;

    // start point
    double X_ini = 0.0, Y_ini = 0.0;

    // current selected waypoint
    double X_final = 0.0, Y_final = 0.0;

    // current pose
    double X_act = 0.0, Y_act = 0.0;

    // maximum accel
    double accel_max = 0.5;

    trajectory_generation::ref_traj ref_traj_obj;
    ros::Publisher trajectory_pub;
    double wp_proxi = 0.5; //(m)

    std::vector<double> X_wp_stack, Y_wp_stack;

    bool start_sim = false;

    int msg_count = 0;
public:

	// constructor
    TrajGen(double);


    // callback functions
    void dynamic_reconfigure_waypoint_callback(trajectory_generation::waypointSetConfig &, uint32_t); 
    void start_sim_callback(const std_msgs::Bool::ConstPtr& );

    // set methods
    void set_wp_proxi(double);
    void set_ini_cond(double,double);
    void set_current_to_ini();
    void set_start_time();
    void set_publisher(ros::Publisher&);

    // process methods
	void select_wp(int);
	void calc_coeff();
    void gen_traj(double);
    
    // publish trajectory
    void publish() const;

    // BOOL methods
    bool is_reached_waypoint() const;
    bool is_start_sim() const;
};


TrajGen::TrajGen(double start_time)
{
    init_time = start_time;
    ref_traj_obj.x = 0.0;
    ref_traj_obj.x_dot = 0.0;
    ref_traj_obj.x_ddot = 0.0;
   
    ref_traj_obj.y = 0.0;
    ref_traj_obj.y_dot = 0.0;
    ref_traj_obj.y_ddot = 0.0;
 
    ref_traj_obj.theta = 0.0;
    ref_traj_obj.theta_dot = 0.0;
    ref_traj_obj.theta_ddot = 0.0;

    ref_traj_obj.header.frame_id = "local_ENU";
}


void TrajGen::dynamic_reconfigure_waypoint_callback(trajectory_generation::waypointSetConfig & config,
                                                        uint32_t level)
{
    X_wp_stack = utility_pkg::string_to_vector<double>(config.X_waypoint);
    Y_wp_stack = utility_pkg::string_to_vector<double>(config.Y_waypoint);

    if (  X_wp_stack.size() != Y_wp_stack.size() )
    {
        ROS_ERROR(" trajectory_generation: x and y waypoints are not of equal size, change them");
        
        X_wp_stack.erase(begin(X_wp_stack), end(X_wp_stack));
        Y_wp_stack.erase(begin(Y_wp_stack), end(Y_wp_stack));
    }
    else if ( X_wp_stack.size() == 1 && X_wp_stack[0] == 0 && Y_wp_stack[0] == 0 )
    { 
        ROS_WARN(" trajectory_generation: waypoint stack is empty, use 'rosservice' to set them "); 
       
        X_wp_stack.erase(begin(X_wp_stack), end(X_wp_stack));
        Y_wp_stack.erase(begin(Y_wp_stack), end(Y_wp_stack));
    }
    else
    {
        ROS_INFO(" trajectory_generation: waypoints received ...");

        //std::cout<<"X-waypoint: "<<X_wp_stack;
        //std::cout<<"Y-waypoint: "<<Y_wp_stack;
    }

}

void TrajGen::start_sim_callback(const std_msgs::Bool::ConstPtr& msg )
{
	start_sim = msg->data;
}


// definitions for above mentioned member functions
void TrajGen::set_wp_proxi(double proximity)
{ wp_proxi = proximity; }


void TrajGen::set_ini_cond(double X_update, double Y_update)
{
    X_ini = X_update; 
    Y_ini = Y_update;
}

void TrajGen::set_current_to_ini()
{
	X_ini = X_act;
	Y_ini = Y_act;
}

void TrajGen::set_start_time()
{ init_time = ros::Time::now().toSec(); }

void TrajGen::set_publisher(ros::Publisher& traj_pub)
{
    trajectory_pub = traj_pub;
}


void TrajGen::select_wp(int wp_index)
{
	if (wp_index > X_wp_stack.size() )
	{
		ROS_WARN("trajectory_generator: cannot increment waypoint selecting last waypoint");
		wp_index = X_wp_stack.size() - 1;
	}
    X_final = X_wp_stack[wp_index];
    Y_final = Y_wp_stack[wp_index];
}

void TrajGen::calc_coeff()
{
    // calculate final time required
   double delta_X = fabs(X_final - X_ini);
   double delta_Y = fabs(Y_final - Y_ini);

   double t_final_X = sqrt(10/(sqrt(3)) * delta_X/accel_max);
   double t_final_Y = sqrt(10/(sqrt(3)) * delta_Y/accel_max);
    
   double tf = std::max(t_final_X, t_final_Y) + std::max(delta_X, delta_Y);

    // calculate X, Y coefficients
    X_coeff[0] = X_ini;
    X_coeff[3] = 10*(X_final - X_ini)/(std::pow(tf,3)); 
    X_coeff[4] = -15*(X_final - X_ini)/(std::pow(tf,4));
    X_coeff[5] = 6*(X_final - X_ini)/(std::pow(tf,5));

    Y_coeff[0] = Y_ini;
    Y_coeff[3] = 10*(Y_final - Y_ini)/(std::pow(tf,3)); 
    Y_coeff[4] = -15*(Y_final - Y_ini)/(std::pow(tf,4));
    Y_coeff[5] = 6*(Y_final - Y_ini)/(std::pow(tf,5));
}

void TrajGen::gen_traj(double time)
{
    double t_rel = (time - init_time);

    // Generate X, Y trajectory
    ref_traj_obj.x          =   X_coeff[0] + X_coeff[1]*t_rel + X_coeff[2]*std::pow(t_rel,2) + 
                                X_coeff[3]*std::pow(t_rel,3) + X_coeff[4]*std::pow(t_rel,4) +
                                X_coeff[5]*std::pow(t_rel,5);

    ref_traj_obj.x_dot       =   X_coeff[1] + 2*X_coeff[2]*t_rel + 3*X_coeff[3]*std::pow(t_rel,2) +
                                4*X_coeff[4]*std::pow(t_rel,3)  + 5*X_coeff[5]*std::pow(t_rel,4);

    ref_traj_obj.x_ddot     =   2*X_coeff[2] + 6*X_coeff[3]*t_rel + 12*X_coeff[4]*std::pow(t_rel,2) +
                                20*X_coeff[5]*std::pow(t_rel,3);


    ref_traj_obj.y          =   Y_coeff[0] + Y_coeff[1]*t_rel + Y_coeff[2]*std::pow(t_rel,2) + 
                                Y_coeff[3]*std::pow(t_rel,3) + Y_coeff[4]*std::pow(t_rel,4) +
                                Y_coeff[5]*std::pow(t_rel,5);

    ref_traj_obj.y_dot      =   Y_coeff[1] + 2*Y_coeff[2]*t_rel + 3*Y_coeff[3]*std::pow(t_rel,2) +
                                4*Y_coeff[4]*std::pow(t_rel,3)  + 5*Y_coeff[5]*std::pow(t_rel,4);

    ref_traj_obj.y_ddot     =   2*Y_coeff[2] + 6*Y_coeff[3]*t_rel + 12*Y_coeff[4]*std::pow(t_rel,2) +
                                20*Y_coeff[5]*std::pow(t_rel,3);

    ref_traj_obj.theta      =   std::atan2(ref_traj_obj.y_dot, ref_traj_obj.x_dot);

    ref_traj_obj.theta_dot  =   0;
    ref_traj_obj.theta_ddot =   0;

    ref_traj_obj.header.seq = msg_count;
    ref_traj_obj.header.stamp = ros::Time::now();

    msg_count++;
}

void TrajGen::publish() const
{
	trajectory_pub.publish(ref_traj_obj); 
}

inline bool TrajGen::is_reached_waypoint() const
{
    double dist = sqrt(std::pow(X_final - X_act,2) + std::pow(Y_final - Y_act,2));
    return (dist <= wp_proxi) ? true : false ;
}

bool TrajGen::is_start_sim() const
{
	if (X_wp_stack.empty())
	{
		ROS_ERROR_ONCE("trajectory_generation: cannot create trajectory, waypoint stack is empty "
					"use dynamic_reconfigure::Server waypointSet_client");
		return false;
	}
	else
	{
		return start_sim;
	}
}


#endif
