/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "rmul_state_machine.hpp"

namespace aruwsrc::algorithms::strategy_state_machine
{
void RMULStateMachine::updateState()
{
    if (autoNavController == nullptr)
    {
        return;
    }

    uint16_t health = refSerial.getRobotData().currentHp;

    // If our health is below a threshold and we were not previously healing, we are now healing
    if (!isHealing && health < HEALING_THRESHOLD)
    {
        path.resetPath();
        for (auto &point : HEALING_PATH)
        {
            path.pushPoint(point);
        }
        isHealing = true;
        return;
    }

    // If our health is above a threshold and we were previously healing, we are now attacking
    if (isHealing && health >= ATTACKING_THRESHOLD)
    {
        path.resetPath();
        for (auto &point : ATTACKING_PATH)
        {
            path.pushPoint(point);
        }
        isHealing = false;
        return;
    }
}

}  // namespace aruwsrc::algorithms::strategy_state_machine
