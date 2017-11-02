/**
 * @brief custom_type_traits header
 * @file custom_type_traits.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */
#ifndef _utility_pkg_CUSTOMTYPETRAITS_H_
#define _utility_pkg_CUSTOMTYPETRAITS_H_

#include <vector>
#include <array>
#include <type_traits>

namespace custom_type_traits{

template<typename T>
struct is_vector : std::false_type{ };

template<typename T>
struct is_vector<std::vector<T>> : std::true_type{ };


template<typename T>
struct is_array : std::false_type{ };

template<typename T, std::size_t array_size>
struct is_array<std::array<T, array_size>> : std::true_type{ };

} // end namespace


#endif
