#ifndef _asl_gremlin_pkg_GETPARAM_H_
#define _asl_gremlin_pkg_GETPARAM_H_

#include <string>
#include <ros/ros.h>

namespace asl_gremlin_pkg{

void throw_warn(const std::string& , int);
void throw_error_and_shutdown(const std::string&, int);
std::string GetParam_with_shutdown(ros::NodeHandle& , const std::string&, int);
std::string GetParam_with_warn(ros::NodeHandle& , const std::string&, int);

} // end namespace {asl_gremlin_pkg}



#endif
