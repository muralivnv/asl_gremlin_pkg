#ifndef STATE_FEEDBACK__VECTOR_ARITHMETIC_H
#define STATE_FEEDBACK__VECTOR_ARITHMETIC_H

#include "custom_type_traits.h"
#include <cassert>
#include <iostream>
#include <algorithm>

#define rm_ref(T) typename std::remove_reference<T>::type

using namespace custom_type_traits;

template<typename T, long unsigned int array_size>
auto create_container(std::array<T,array_size>& arr)
{
    return std::array<T,array_size>();
}

template<typename T>
auto create_container(std::vector<T>& vec)
{
    return std::vector<T>(vec.size());
}
/*
 *              OPERATOR (+)
 */
template<typename T, typename S>
auto operator+(T&& container1, S&& container2)
    -> typename std::enable_if< ((is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                                (is_vector<rm_ref(S)>::value || is_array<rm_ref(S)>::value)), rm_ref(T)>::type
{
    assert (container1.size() == container2.size());

    rm_ref(T) result_container = create_container(container1);

    std::transform(container1.begin(), container1.end(),
                    container2.begin(), result_container.begin(),
                    [](auto n1, auto n2){return n1 + n2;});

    return result_container;
}

template<typename T, typename S>
auto operator+(T&& container, S&& scalar)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                (std::is_floating_point<rm_ref(S)>::value || std::is_integral<rm_ref(S)>::value), rm_ref(T)>::type
{

    rm_ref(T) result_container = create_container(container);

    std::transform(container.begin(),container.end(),
                    result_container.begin(),
                    [=](auto n){ return n += scalar; });

    return result_container;
}


template<typename T, typename S>
auto operator+(S&& scalar, T& container)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                (std::is_floating_point<rm_ref(S)>::value || std::is_integral<rm_ref(S)>::value), rm_ref(T)>::type
{
    return container + scalar;
}


/*
 *              OPERATOR (-)
 */
template<typename T, typename S>
auto operator-(T&& container1, S&& container2)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                                (is_vector<rm_ref(S)>::value || is_array<rm_ref(S)>::value), rm_ref(T)>::type
{
    assert (container1.size() == container2.size());

    rm_ref(T) result_container = create_container(container1);

    std::transform(container1.begin(), container1.end(),
                    container2.begin(), result_container.begin(),
                    [](auto n1, auto n2){return n1 - n2;});

    return result_container;
}

template<typename T, typename S>
auto operator-(T&& container, S&& scalar)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                (std::is_floating_point<rm_ref(S)>::value || std::is_integral<rm_ref(S)>::value), rm_ref(T)>::type
{
    rm_ref(T) result_container = create_container(container);

    std::transform(container.begin(),container.end(),
                    result_container.begin(),
                    [=](auto n){ return n -= scalar; });

    return result_container;
}


template<typename T, typename S>
auto operator-(S&& scalar, T& container)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                (std::is_floating_point<rm_ref(S)>::value || std::is_integral<rm_ref(S)>::value), rm_ref(T)>::type
{
    return container - scalar;
}


/*
 *              OPERATOR (*)
 */
template<typename T, typename S>
auto operator*(T&& container1, S&& container2)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                                (is_vector<rm_ref(S)>::value || is_array<rm_ref(S)>::value), rm_ref(T)>::type
{
    assert (container1.size() == container2.size());

    rm_ref(T) result_container = create_container(container1);

    std::transform(container1.begin(), container1.end(),
                    container2.begin(), result_container.begin(),
                    [](auto n1, auto n2){return n1 * n2;});

    return result_container;
}

template<typename T, typename S>
auto operator*(T&& container, S&& scalar)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                (std::is_floating_point<rm_ref(S)>::value || std::is_integral<rm_ref(S)>::value), rm_ref(T)>::type
{
    rm_ref(T) result_container = create_container(container);

    std::transform(container.begin(),container.end(),
                    result_container.begin(),
                    [=](auto n){ return n *= scalar; });

    return result_container;
}


template<typename T, typename S>
auto operator*(S&& scalar, T&& container)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                (std::is_floating_point<rm_ref(S)>::value || std::is_integral<rm_ref(S)>::value), rm_ref(T)>::type
{
    return container * scalar;
}


/*
 *              OPERATOR (/)
 */
template<typename T, typename S>
auto operator/(T&& container1, S&& container2)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                                (is_vector<rm_ref(S)>::value || is_array<rm_ref(S)>::value), rm_ref(T)>::type
{
    assert (container1.size() == container2.size());

    rm_ref(T) result_container = create_container(container1);

    std::transform(container1.begin(), container1.end(),
                    container2.begin(), result_container.begin(),
                    [](auto n1, auto n2){return n1 / n2;});

    return result_container;
}

template<typename T, typename S>
auto operator/(T&& container, S&& scalar)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                (std::is_floating_point<rm_ref(S)>::value || std::is_integral<rm_ref(S)>::value), rm_ref(T)>::type
{
    rm_ref(T) result_container = create_container(container);

    std::transform(container.begin(),container.end(),
                    result_container.begin(),
                    [=](auto n){ return n /= scalar; });

    return result_container;
}


template<typename T, typename S>
auto operator/(S&& scalar, T& container)
    -> typename std::enable_if< (is_vector<rm_ref(T)>::value || is_array<rm_ref(T)>::value) &&
                (std::is_floating_point<rm_ref(S)>::value || std::is_integral<rm_ref(S)>::value), rm_ref(T)>::type
{
    return container / scalar;
}

#endif
