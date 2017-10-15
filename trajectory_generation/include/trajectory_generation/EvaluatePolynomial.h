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

double expand_diff_coeff(int N, int order)
{
    if ( N < 0)
    { return 0; }
    else if (order == 0)
    { return 1; }
    else
    { return N*expand_diff_coeff(N-1, order-1); }
}

template <typename T>
double evaluatePolynomial(int N, double t, const T& arr, orderOfDiff order)
{
    if ( N == 0 )
    { return order > 0 ? 0 : arr[0]; }

    double differentiation_coeff = expand_diff_coeff(N, order);
    if ( N == order )
    { return differentiation_coeff * arr[N]; }
    else
    {
        return differentiation_coeff * arr[N] * std::pow(t, N - order)+
            evaluatePolynomial (N-1, t,arr,order);
    }
}

template<int N, typename T>
double eval_poly(double time, const T& coefficients, orderOfDiff order)
{
    try{
        if ( coefficients.size() < N + 1 )
            throw "insufficient coefficients for polynomial of order N -> in";

        return evaluatePolynomial(N, time, coefficients, order);
    }
    catch (const char* msg){
        ROS_ERROR("%s\n %s at %d", msg, __FILE__, __LINE__);
    }
}

#endif
