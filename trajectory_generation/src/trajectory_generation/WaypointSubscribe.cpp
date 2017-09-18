#include <trajectory_generation/WaypointSubscribe.h>

std::vector<double> WaypointSubscribe::x_waypoints_ = {0};
std::vector<double> WaypointSubscribe::y_waypoints_ = {0};


WaypointSubscribe::WaypointSubscribe(ros::NodeHandle& nh)
{
    fun_ = boost::bind(&WaypointSubscribe::dynamic_reconfigure_waypointSet_callback,
                       _1, _2);

    dr_wp_srv_.setCallback(fun_);
}

void WaypointSubscribe::dynamic_reconfigure_waypointSet_callback(trajectory_generation::waypointSetConfig & config,  uint32_t level)
{
    x_waypoints_ = utility_pkg::string_to_vector<double>(config.X_waypoint);
    y_waypoints_ = utility_pkg::string_to_vector<double>(config.Y_waypoint);

    if (  x_waypoints_.size() != y_waypoints_.size() )
    {
        ROS_ERROR(" trajectory_generation: x and y waypoints are not of equal size, change them");
        
        x_waypoints_.erase(begin(x_waypoints_), end(x_waypoints_));
        y_waypoints_.erase(begin(y_waypoints_), end(y_waypoints_));
    }
    else if ( x_waypoints_.size() == 1 && x_waypoints_[0] == 0 && y_waypoints_[0] == 0 )
    { 
        ROS_WARN(" trajectory_generation: waypoint stack is empty, use 'rosservice' to set them "); 
       
        x_waypoints_.erase(begin(x_waypoints_), end(x_waypoints_));
        y_waypoints_.erase(begin(y_waypoints_), end(y_waypoints_));
    }
    else
    {
        ROS_INFO(" trajectory_generation: waypoints received ...");
        
        std::cout << "X_waypoints:  ";
        utility_pkg::print_stl_container(x_waypoints_);
        
        std::cout << "\nY_waypoints:  ";
        utility_pkg::print_stl_container(y_waypoints_);
    }

}

std::vector<double> WaypointSubscribe::get_current_waypoint()
{
    return {x_waypoints_[current_waypoint_ptr_],
                y_waypoints_[current_waypoint_ptr_] };
}


std::vector<double> WaypointSubscribe::get_next_waypoint()
{
    ++current_waypoint_ptr_;
    return {x_waypoints_[current_waypoint_ptr_],
                y_waypoints_[current_waypoint_ptr_] };
}

