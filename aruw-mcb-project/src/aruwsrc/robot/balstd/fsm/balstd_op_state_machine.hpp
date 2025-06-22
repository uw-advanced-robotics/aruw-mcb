/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef BALSTD_OP_STATE_MACHINE_HPP_
#define BALSTD_OP_STATE_MACHINE_HPP_

#include "tap/algorithms/transforms/transform.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/robot/balstd/chassis/balstd_chassis_state.hpp"

#include "balstd_op_states.hpp"

namespace aruwsrc::balstd
{

class BalstdOpStateMachine : public tap::control::Subsystem
{
public:
    BalstdOpStateMachine(tap::Drivers* drivers, const chassis::BalstdChassisState& chassisState);

    void initialize() override;

    void refresh() override;

    inline const BalstdOpState& getCurrentState() const { return currentState; }

private:
    BalstdOpState currentState;
    const chassis::BalstdChassisState& chassisState;

    static constexpr float CONTROLLABLE_CHASSIS_PITCH_LIMIT = M_PI_4;
};

}  // namespace aruwsrc::balstd

#endif  // BALSTD_OP_STATE_MACHINE_HPP_