/**
 * @brief custom string manipulations header
 * @file str_manip.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _utility_pkg_STRMANIP_H_
#define _utility_pkg_STRMANIP_H_

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include "tinyexpr.h"

namespace utility_pkg {

std::vector<std::string> split(const std::string& S, char delim)
{
    std::stringstream str_stream(S);
    std::string str_item;
    std::vector<std::string> splitted_strs;

    while ( std::getline(str_stream, str_item, delim) )
    { splitted_strs.push_back(str_item); }

    return splitted_strs;
}

template <typename T>
std::vector<T> string_to_vector(std::string& S)
{
    auto splitted_strs = split(S, ',');
    
    std::vector<T> Vec(splitted_strs.size(),0);
    std::transform(splitted_strs.begin(), splitted_strs.end(), std::begin(Vec),
                    [](auto item){ return te_interp(item.c_str(),0); });
    return Vec;
}

template<typename type, typename ... Args, template <typename, typename ...> class STL_Container>
void print_stl_container(const STL_Container<type, Args...>& stl_container)
{
    std::cout << "{";
    for (typename STL_Container<type, Args...>::const_iterator c_it = begin(stl_container);
            c_it != end(stl_container); ++c_it)
    {
        std::cout << *c_it;
        if ( c_it < end(stl_container)-1)
        { std::cout << ", "; }
    }
    std::cout << "}\n";
}

}; // end namespace {utility_pkg}

template <typename T, typename S>
std::ostream& operator<<(std::ostream& os, std::pair<T,S>& stl_pair)
{
	os << stl_pair.first << ' ' << stl_pair.second<<'\n';
	return os;
}


#endif
