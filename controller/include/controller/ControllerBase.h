#ifndef CONTROLLER__CONTROLLER_BASE_H
#define CONTROLLER__CONTROLLER_BASE_H

#include <iostream>
#include <array>

namespace controller{


template<typename ref_state_type, typename act_state_type>
class ControllerBase{

    public:
        virtual ~ControllerBase();
        virtual void calculate_control_action(const ref_state_type&, const act_state_type&) = 0;

        virtual std::array<double,2> get_control_action() = 0;
};

} // end namespace
#endif
