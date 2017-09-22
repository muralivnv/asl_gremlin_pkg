#ifndef UTILITY_PKG__PARAM_ERROR_UTIL_H
#define UTILITY_PKG__PARAM_ERROR_UTIL_H

#include <ros/ros.h>

namespace utility_pkg{

void throw_error(const std::string&, int);

void throw_warn(const std::string&, int);

void throw_warn(const std::string&, int, const std::string& msg);

void throw_error_and_shutdown(const std::string&, int);

} //end namespace{utility_pkg}


#endif
