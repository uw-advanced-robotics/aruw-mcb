#ifndef STATE_MACHINE_HPP_
#define STATE_MACHINE_HPP_

#include "balstd_states.hpp"

namespace aruwsrc::control::balstd
{

class BalstdStateMachine
{
public:
    BalstdStateMachine();

private:
    BalstdState currentState;
};

}  // namespace aruwsrc::control::balstd

#endif  // STATE_MACHINE_HPP_