/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "otto_chassis_velocity_displacement_2d_observer.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "modm/math/geometry/vector.hpp"
#include "modm/math/matrix.hpp"

namespace aruwsrc::algorithms::odometry
{
OttoChassisVelocityDisplacement2DObserver::OttoChassisVelocityDisplacement2DObserver(
    tap::control::chassis::ChassisSubsystemInterface* chassis)
    : chassis(chassis),
      absoluteDisplacement(0.0f, 0.0f, 0.0f)
{
}

void OttoChassisVelocityDisplacement2DObserver::update()
{
    uint32_t currTime = tap::arch::clock::getTimeMicroseconds();
    if (chassis->allMotorsOnline())
    {
        // Check whether this is first update
        if (prevTime != 0)
        {
            // This is subsequent update, data is now definitely valid
            dataValid = true;
            modm::Matrix<float, 3, 1> chassisRelativeVelocity =
                chassis->getActualVelocityChassisRelative();

            // initially set velocity to chassis relative velocity to be used in displacement
            // computations
            velocity.set(chassisRelativeVelocity[0][0], chassisRelativeVelocity[1][0]);

            // m/s * us * 1s / 1'000'000 us
            modm::Vector<float, 2> displacementThisTick =
                velocity * (static_cast<float>(currTime - prevTime) / 1'000'000.0f);
            absoluteDisplacement.move(displacementThisTick);
        }
        prevTime = currTime;
    }
    else
    {
        // motors offline, reset state
        prevTime = 0;
    }
}

bool OttoChassisVelocityDisplacement2DObserver::getVelocityChassisDisplacement(
    modm::Vector3f* const velocity,
    modm::Vector3f* const displacement) const
{
    velocity->setX(this->velocity.getX());
    velocity->setY(this->velocity.getY());
    velocity->setZ(0.0f);

    displacement->setX(absoluteDisplacement.getX());
    displacement->setY(absoluteDisplacement.getY());
    displacement->setZ(0.0f);

    return dataValid;
}
}  // namespace aruwsrc::algorithms::odometry
