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

#include "otto_chassis_velocity_displacement_2d_getter.hpp"

#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "modm/math/matrix.hpp"

namespace aruwsrc::control::odometry
{
OttoChassisVelocityDisplacement2DGetter::OttoChassisVelocityDisplacement2DGetter(
    aruwsrc::chassis::ChassisSubsystem* chassis)
    : chassis(chassis)
{
}

bool OttoChassisVelocityDisplacement2DGetter::getChassisDisplacement(float* x, float* y, float* z)
{
    // 2D displacement getter, so z-component defaults to 0.
    *z = 0.0f;

    if (!chassis->areAllMotorsOnline() || prevTime == 0)
    {
        // First time being called or some motors offline. Either way
        // update prevTime and return false with all 0's.
        prevTime = tap::arch::clock::getTimeMilliseconds();
        *x = 0;
        *y = 0;
        return false;
    }
    else
    {
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        uint32_t dt = currTime - prevTime;
        prevTime = currTime;

        modm::Matrix<float, 3, 1> chassisVelocity = chassis->getActualVelocityChassisRelative();
        *x = chassisVelocity[0][0] * dt / 1000.0f;
        // Negate chassis y velocity component as the chassis velocity uses a coordinate frame
        // where positive y is to the right instead of the left.
        *y = -chassisVelocity[1][0] * dt / 1000.0f;
        return true;
    }
}
}  // namespace aruwsrc::control::odometry
