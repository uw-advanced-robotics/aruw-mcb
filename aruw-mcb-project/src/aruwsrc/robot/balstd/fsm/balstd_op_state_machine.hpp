#ifndef BALSTD_OP_STATE_MACHINE_HPP_
#define BALSTD_OP_STATE_MACHINE_HPP_

#include "balstd_states.hpp"

namespace aruwsrc::control::balstd
{

class BalstdOpStateMachine
{
public:
    BalstdOpStateMachine();

private:
    BalstdOpState currentState;
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_OP_STATE_MACHINE_HPP_