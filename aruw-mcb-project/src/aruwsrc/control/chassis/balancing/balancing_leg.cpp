/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balancing_leg.hpp"
#include <assert.h>

namespace aruwsrc
{
namespace chassis
{
    BalancingLeg::BalancingLeg(
        tap::Drivers* drivers,
        aruwsrc::control::motion::FiveBarLinkage* fivebar,
        tap::motor::MotorInterface* wheelMotor,
        const float wheelRadius
    ) : fivebar(fivebar),
        driveWheel(wheelMotor),
        WHEEL_RADIUS(wheelRadius)
    {
        assert(fivebar != nullptr);
        assert(wheelMotor != nullptr);
    }

    void BalancingLeg::update()
    {
        // 1. Update Current Output Values
        fblCurrentPosition = fivebar->getCurrentPosition();
        zCurrent = -fblCurrentPosition.getY();
        tl_Current = atan2(zCurrent, -fblCurrentPosition.getX());
        vCurrent = WHEEL_RADIUS*driveWheel->getShaftRPM();

        // 2. Apply Control Law


        // 3. Send New Output Values
        
    }
}
}
