#ifndef BALSTD_OP_STATE_MACHINE_HPP_
#define BALSTD_OP_STATE_MACHINE_HPP_

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/robot/balstd/chassis/balstd_chassis_state.hpp"

#include "balstd_op_states.hpp"

namespace aruwsrc::control::balstd
{

class BalstdOpStateMachine : public tap::control::Subsystem
{
public:
    BalstdOpStateMachine(tap::Drivers* drivers, const BalstdChassisState& chassisState);

    void initialize() override;

    void refresh() override;

    inline const BalstdOpState& getCurrentState() const { return currentState; }

private:
    BalstdOpState currentState;
    const BalstdChassisState& chassisState;

    static constexpr float CONTROLLABLE_CHASSIS_PITCH_LIMIT = M_PI_4;
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_OP_STATE_MACHINE_HPP_