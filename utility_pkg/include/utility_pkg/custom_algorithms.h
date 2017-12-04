/**
 * @brief Custom algorithms header
 * @file custom_algorithms.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _utility_pkg_CUSTOMALGORITHMS_H_
#define _utility_pkg_CUSTOMALGORITHMS_H_

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace utility_pkg{
namespace custom_algorithms{

template<typename ContainerType, typename ValueType>
int get_lower_index(const ContainerType& container, ValueType value)
{
    try
    {
        if (value < *(container.begin()) || value > *(container.end()-1))
        { throw std::length_error("value is out-of-bounds on either side of range"); }

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

template<typename YContainerType, typename XContainerType, typename IndexType, typename ValueType>
auto linear_interpolate(const YContainerType& y_container, const XContainerType& x_container, IndexType index, ValueType x)
{
    auto y0 = *(y_container.begin() + index);
    auto y1 = *(y_container.begin() + index + 1);

    auto x0 = *(x_container.begin() + index);
    auto x1 = *(x_container.begin() + index + 1);

    return y0 + (x - x0)*(y1 - y0)/(x1 - x0);
}

template<typename XContainerType, typename YContainerType, typename ValueType>
ValueType lookup_table(const XContainerType& x_container, const YContainerType& y_container, const ValueType& x_val)
{

    int idx = get_lower_index(x_container, x_val);
    ValueType interpolated_data = linear_interpolate(y_container, x_container, idx, x_val);
    return interpolated_data;
}

} // end namespace { custom_algorithms }
} // end namespace { utility_pkg }



#endif
