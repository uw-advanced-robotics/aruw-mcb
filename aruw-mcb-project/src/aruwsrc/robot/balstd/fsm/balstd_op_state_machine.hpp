#ifndef BALSTD_OP_STATE_MACHINE_HPP_
#define BALSTD_OP_STATE_MACHINE_HPP_

#include "balstd_op_states.hpp"

namespace aruwsrc::control::balstd
{

class BalstdOpStateMachine
{
public:
    BalstdOpStateMachine();

    void update();

private:
    BalstdOpState currentState;
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_OP_STATE_MACHINE_HPP_