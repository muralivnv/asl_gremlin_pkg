#ifndef _trajectory_generation_EVALUATEPOLYNOMIAL_H_
#define _trajectory_generation_EVALUATEPOLYNOMIAL_H_

#include <iostream>
#include <array>
#include <cmath>
#include <ros/ros.h>
#include <stdexcept>

enum orderOfDiff{
    position,
    velocity,
    acceleration,
    jerk,
    snap
};


double expand_Nth_differentiation_coeff(int N, int order)
{
    if ( N < 0)
    { return 0; }

    else if (order == 0)
    { return 1; }

    else
    { return N*expand_Nth_differentiation_coeff(N-1, order-1); }
}

template <int N, std::size_t array_size>
struct evaluate_Nth_order_polynomial{
    double operator()(double t, std::array<double, array_size>& arr, orderOfDiff order)
    {
        double differentiation_coeff = expand_Nth_differentiation_coeff(N, order);

        evaluate_Nth_order_polynomial <N - 1, array_size> polynomial_obj;

        return differentiation_coeff * arr[N] * std::pow(t, N - order) + 
                polynomial_obj(t,arr,order);
    }
};


template <std::size_t array_size>
struct evaluate_Nth_order_polynomial <0, array_size> {
    double operator()(double t, std::array<double, array_size>& arr, orderOfDiff order)
    {
        return order > 0 ? 0 : arr[0];
    }
};

template<int N, std::size_t coeff_size>
double get_Nth_order_polynomial(double time, std::array<double, coeff_size>& coefficients, orderOfDiff order)
{
    try{
        if (coeff_size < N + 1)
            throw "insufficient coefficients for polynomial of order N -> in";

        evaluate_Nth_order_polynomial <N, coeff_size> polynomial_obj;
        return polynomial_obj(time, coefficients, order);
    }
    catch (const char* msg){
        ROS_ERROR("%s\n", msg);
    }
}

#endif
