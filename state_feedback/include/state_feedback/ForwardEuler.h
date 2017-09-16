#ifndef STATE_FEEDBACK__FORWARD_EULER_H
#define STATE_FEEDBACK__FORWARD_EULER_H

#include <iostream>
#include <array>
#include "vector_arithmetic.h"

namespace state_feedback{
namespace numerical_diff{

double forward_euler_step_size = 0.1;

template<   typename System,
            typename State_type,
            typename time_type,
            typename parameter_type = void >
State_type forwardEuler_integration(System system, State_type& initial_state, time_type start_time,
                              time_type end_time, parameter_type* param = nullptr)
{
    State_type x_current = initial_state;

    for (time_type t = start_time + forward_euler_step_size;
              t <= end_time; t += forward_euler_step_size)
    {
        x_current = x_current + system(t, x_current, *param);
    }

    return x_current;
}


} // end namespace ( numerical_diff )
} // end namespace ( state_feedback )

#endif
