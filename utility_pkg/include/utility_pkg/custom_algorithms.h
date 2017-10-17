#ifndef _utility_pkg_CUSTOMALGORITHMS_H_
#define _utility_pkg_CUSTOMALGORITHMS_H_

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace utility_pkg{
namespace custom_algorithms{

template<typename T, typename S>
int get_lower_index(const T& container, S value)
{
    try{
        if (value < *(container.begin()) || value > *(container.end()-1))
        {
            throw std::length_error("value is out-of-bounds on either side of range");
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
        return 0;
    }
}

template<typename T, typename S, typename N, typename val>
auto linear_interpolate(const T& y_container, const S& x_container, N index, val x)
{
    auto y0 = *(y_container.begin() + index);
    auto y1 = *(y_container.begin() + index + 1);

    auto x0 = *(x_container.begin() + index);
    auto x1 = *(x_container.begin() + index + 1);

    return y0 + (x - x0)*(y1 - y0)/(x1 - x0);
}

template<typename T, typename S, typename val_t>
val_t lookup_table(const T& x_container, const S& y_container, const val_t& x_val)
{

    int idx = get_lower_index(x_container, x_val);
    val_t interpolated_data = linear_interpolate(y_container, x_container, idx, x_val);
}

} // end namespace { custom_algorithms }
} // end namespace { utility_pkg }



#endif
