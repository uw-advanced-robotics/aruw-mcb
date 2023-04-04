/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balancing_chassis_subsystem.hpp"

namespace aruwsrc::chassis
{
    BalancingChassisSubsystem::BalancingChassisSubsystem(
        tap::Drivers* drivers,
        BalancingLeg& leftLeg,
        BalancingLeg& rightLeg
    ) : Subsystem(drivers),
        leftLeg(leftLeg),
        rightLeg(rightLeg)
    {

    }

    void BalancingChassisSubsystem::initialize()
    {
        desiredX = 0;
        desiredR = 0;
        desiredZ = 0;
    }

    void BalancingChassisSubsystem::refresh()
    {
        // 1. Update yaw and roll values
        

        // 2. Apply scaling and/or control laws to yaw and roll values
        float yawAdjustment = 0;
        float rollAdjustment = 0;

        // 3. Set each side's actuators to compensate appropriate for yaw and roll error
        leftLeg.setDesiredHeight(desiredZ + rollAdjustment);
        rightLeg.setDesiredHeight(desiredZ - rollAdjustment);

        if (desiredX + yawAdjustment > MAX_WHEEL_SPEED)
        {
            desiredX = MAX_WHEEL_SPEED - yawAdjustment;
        }
        leftLeg.setDesiredTranslationSpeed(desiredX + yawAdjustment);
        rightLeg.setDesiredTranslationSpeed(desiredX - yawAdjustment);
    }

    void BalancingChassisSubsystem::runHardwareTests()
    {

    }
}   // namespace aruwsrc::chassis
