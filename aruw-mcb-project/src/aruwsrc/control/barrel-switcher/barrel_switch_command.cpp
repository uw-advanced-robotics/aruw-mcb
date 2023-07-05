/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "barrel_switch_command.hpp"

namespace aruwsrc::control
{
void BarrelSwitchCommand::initialize()
{
    controlState = SwitchingControlState::AUTOMATIC;
}

void BarrelSwitchCommand::execute()
{
    switch (controlState)
    {
        case SwitchingControlState::USING_LEFT:
            if (barrelSwitcher->getBarrelState() != BarrelState::USING_LEFT_BARREL)
            {
                barrelSwitcher->useLeft();
            }
            break;
        case SwitchingControlState::USING_RIGHT:
            if (barrelSwitcher->getBarrelState() != BarrelState::USING_RIGHT_BARREL)
            {
                barrelSwitcher->useRight();
            }
            break;
        case SwitchingControlState::AUTOMATIC:
            if (barrelSwitcher->getBarrelState() == BarrelState::IDLE)
            {
                barrelSwitcher->useRight();
            }
            if (barrelSwitcher->getBarrelState() == BarrelState::USING_LEFT_BARREL &&
                !heatTrackerLeft.enoughHeatToLaunchProjectile())
            {
                barrelSwitcher->useRight();
            }
            else if (
                barrelSwitcher->getBarrelState() == BarrelState::USING_RIGHT_BARREL &&
                !heatTrackerRight.enoughHeatToLaunchProjectile())
            {
                barrelSwitcher->useLeft();
            }
            break;
        case SwitchingControlState::NUM_STATES:
            break;
    }
}

void BarrelSwitchCommand::setControlState(SwitchingControlState state)
{
    if (state < SwitchingControlState::NUM_STATES)
    {
        this->controlState = state;
    }
}

void BarrelSwitchCommand::end(bool)
{
    barrelSwitcher->stop();
}
}  // namespace aruwsrc::control
