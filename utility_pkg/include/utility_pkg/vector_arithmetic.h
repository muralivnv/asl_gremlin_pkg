#ifndef _utility_pkg_VECTORARITHMETIC_H_
#define _utility_pkg_VECTORARITHMETIC_H_

#include "custom_type_traits.h"
#include <cassert>
#include <iostream>
#include <algorithm>

#define RM_REF(T) typename std::remove_reference<T>::type

using namespace custom_type_traits;

template<typename T, std::size_t array_size>
std::array<T, array_size> create_container(std::array<T,array_size>& arr)
{
    return std::array<T,array_size>();
}

template<typename T>
std::vector<T> create_container(std::vector<T>& vec)
{
    return std::vector<T>(vec.size());
}
/*
 *              OPERATOR (+)
 */
template<typename T, typename S>
auto operator+(T&& container1, S&& container2)
    -> typename std::enable_if< ((is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                                (is_vector<RM_REF(S)>::value || is_array<RM_REF(S)>::value)), RM_REF(T)>::type
{
    assert (container1.size() == container2.size());

    RM_REF(T) result_container = create_container(container1);

    std::transform(container1.begin(), container1.end(),
                    container2.begin(), result_container.begin(),
                    [](auto n1, auto n2){return n1 + n2;});

    return result_container;
}

template<typename T, typename S>
auto operator+(T&& container, S&& scalar)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                (std::is_floating_point<RM_REF(S)>::value || std::is_integral<RM_REF(S)>::value), RM_REF(T)>::type
{

    RM_REF(T) result_container = create_container(container);

    std::transform(container.begin(),container.end(),
                    result_container.begin(),
                    [=](auto n){ return n += scalar; });

    return result_container;
}


template<typename T, typename S>
auto operator+(S&& scalar, T& container)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                (std::is_floating_point<RM_REF(S)>::value || std::is_integral<RM_REF(S)>::value), RM_REF(T)>::type
{
    return container + scalar;
}


/*
 *              OPERATOR (-)
 */
template<typename T, typename S>
auto operator-(T&& container1, S&& container2)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                                (is_vector<RM_REF(S)>::value || is_array<RM_REF(S)>::value), RM_REF(T)>::type
{
    assert (container1.size() == container2.size());

    RM_REF(T) result_container = create_container(container1);

    std::transform(container1.begin(), container1.end(),
                    container2.begin(), result_container.begin(),
                    [](auto n1, auto n2){return n1 - n2;});

    return result_container;
}

template<typename T, typename S>
auto operator-(T&& container, S&& scalar)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                (std::is_floating_point<RM_REF(S)>::value || std::is_integral<RM_REF(S)>::value), RM_REF(T)>::type
{
    RM_REF(T) result_container = create_container(container);

    std::transform(container.begin(),container.end(),
                    result_container.begin(),
                    [=](auto n){ return n -= scalar; });

    return result_container;
}


template<typename T, typename S>
auto operator-(S&& scalar, T& container)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                (std::is_floating_point<RM_REF(S)>::value || std::is_integral<RM_REF(S)>::value), RM_REF(T)>::type
{
    return container - scalar;
}


/*
 *              OPERATOR (*)
 */
template<typename T, typename S>
auto operator*(T&& container1, S&& container2)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                                (is_vector<RM_REF(S)>::value || is_array<RM_REF(S)>::value), RM_REF(T)>::type
{
    assert (container1.size() == container2.size());

    RM_REF(T) result_container = create_container(container1);

    std::transform(container1.begin(), container1.end(),
                    container2.begin(), result_container.begin(),
                    [](auto n1, auto n2){return n1 * n2;});

    return result_container;
}

template<typename T, typename S>
auto operator*(T&& container, S&& scalar)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                (std::is_floating_point<RM_REF(S)>::value || std::is_integral<RM_REF(S)>::value), RM_REF(T)>::type
{
    RM_REF(T) result_container = create_container(container);

    std::transform(container.begin(),container.end(),
                    result_container.begin(),
                    [=](auto n){ return n *= scalar; });

    return result_container;
}


template<typename T, typename S>
auto operator*(S&& scalar, T&& container)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                (std::is_floating_point<RM_REF(S)>::value || std::is_integral<RM_REF(S)>::value), RM_REF(T)>::type
{
    return container * scalar;
}


/*
 *              OPERATOR (/)
 */
template<typename T, typename S>
auto operator/(T&& container1, S&& container2)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                                (is_vector<RM_REF(S)>::value || is_array<RM_REF(S)>::value), RM_REF(T)>::type
{
    assert (container1.size() == container2.size());

    RM_REF(T) result_container = create_container(container1);

    std::transform(container1.begin(), container1.end(),
                    container2.begin(), result_container.begin(),
                    [](auto n1, auto n2){return n1 / n2;});

    return result_container;
}

template<typename T, typename S>
auto operator/(T&& container, S&& scalar)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                (std::is_floating_point<RM_REF(S)>::value || std::is_integral<RM_REF(S)>::value), RM_REF(T)>::type
{
    RM_REF(T) result_container = create_container(container);

    std::transform(container.begin(),container.end(),
                    result_container.begin(),
                    [=](auto n){ return n /= scalar; });

    return result_container;
}


template<typename T, typename S>
auto operator/(S&& scalar, T& container)
    -> typename std::enable_if< (is_vector<RM_REF(T)>::value || is_array<RM_REF(T)>::value) &&
                (std::is_floating_point<RM_REF(S)>::value || std::is_integral<RM_REF(S)>::value), RM_REF(T)>::type
{
    return container / scalar;
}

#endif
