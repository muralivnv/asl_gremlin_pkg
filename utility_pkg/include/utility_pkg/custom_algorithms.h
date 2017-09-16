#ifndef UTILITY_PKG__CUSTOM_ALGORITHMS_H
#define UTILITY_PKG__CUSTOM_ALGORITHMS_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace utility_pkg{
    namespace custom_algorithms{

template<typename T, typename S>
int get_lower_index(T& container, S value)
{
    try{

        if (value < *(container.begin()) || value > *(container.end()-1))
        {
            throw std::length_error("value is out-of-bounds on either side of range");
            return 0;
        }
        else
        {
            auto val_it = std::lower_bound(container.begin(), container.end(), value);
            
            int idx = ( val_it - container.begin() );
            
            idx -= 1;
            return idx < 0 ? 0 : idx;
        }
    }
    catch(std::length_error& msg)
    {
        ROS_WARN("%s", msg.what());
    }
}


template<typename T, typename S, typename N, typename val>
auto linear_interpolate(T& y_container, S& x_container, N index, val x)
{
    auto y0 = *(y_container.begin() + index);
    auto y1 = *(y_container.begin() + index + 1);

    auto x0 = *(x_container.begin() + index);
    auto x1 = *(x_container.begin() + index + 1);
    
    auto y  = y0 + (x - x0)*(y1 - y0)/(x1 - x0);

    return y;
}

    } // end namespace{custom_algorithms}
} // end namespace{utility_pkg}



#endif
