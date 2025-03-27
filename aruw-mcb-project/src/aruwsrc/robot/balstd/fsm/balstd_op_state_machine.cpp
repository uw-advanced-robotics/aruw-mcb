#include "balstd_op_state_machine.hpp"

namespace aruwsrc::control::balstd
{

BalstdOpStateMachine::BalstdOpStateMachine(
    tap::Drivers* drivers,
    const BalstdChassisState& chassisState)
    : Subsystem(drivers),
      currentState(BalstdOpState::UNKNOWN),
      chassisState(chassisState)
{
}

void BalstdOpStateMachine::initialize()
{
    // assume we startup fallen forward
    currentState = BalstdOpState::FALLEN_FORWARD;
}

void BalstdOpStateMachine::refresh()
{
    // TODO: consider imu calibrate state when allowing get up

    float chassisPitch = chassisState.leftLegState;

    // TODO: use pendulum angle
    if (chassisPitch > CONTROLLABLE_CHASSIS_PITCH_LIMIT)
    {
        currentState = BalstdOpState::FALLEN_FORWARD;
    }
    else if (chassisPitch < -CONTROLLABLE_CHASSIS_PITCH_LIMIT)
    {
        currentState = BalstdOpState::FALLEN_BACKWARD;
    }
    else  // within controllable chassis pitch range
    {
        if (currentState == BalstdOpState::GETTING_UP_BACKWARD ||
            currentState == BalstdOpState::GETTING_UP_FORWARD)
        {
            currentState = BalstdOpState::BALANCING;
        }
    }
}

}  // namespace aruwsrc::control::balstd
