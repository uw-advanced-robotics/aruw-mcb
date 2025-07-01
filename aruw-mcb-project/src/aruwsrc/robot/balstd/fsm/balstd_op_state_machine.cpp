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

#include "balstd_op_state_machine.hpp"

namespace aruwsrc::balstd
{
BalstdOpStateMachine::BalstdOpStateMachine(
    tap::Drivers* drivers,
    const chassis::BalstdChassisState& chassisState)
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

    float chassisPitch = 0;  // chassisState.leftLegState;

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

}  // namespace aruwsrc::balstd
