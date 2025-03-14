#include "balstd_op_state_machine.hpp"

namespace aruwsrc::control::balstd
{

BalstdOpStateMachine::BalstdOpStateMachine() : currentState(BalstdOpState::UNKNOWN) {}

void BalstdOpStateMachine::update()
{
    //
}

}  // namespace aruwsrc::control::balstd
