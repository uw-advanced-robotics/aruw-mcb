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

#include "otto_velocity_odometry_2d_subsystem.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::algorithms::odometry
{
// Yep that's all this constructor does, is construct the right getters and pass
// their pointers to the base class (not necessarily in that order)
OttoVelocityOdometry2DSubsystem::OttoVelocityOdometry2DSubsystem(
    aruwsrc::Drivers* drivers,
    const aruwsrc::control::turret::TurretSubsystem& turret,
    tap::control::chassis::ChassisSubsystemInterface* chassis)
    : Subsystem(drivers),
      Odometry2DTracker(&orientationObserver, &displacementObserver),
      orientationObserver(drivers, turret),
      displacementObserver(chassis)
{
}

void OttoVelocityOdometry2DSubsystem::refresh()
{
    // Update displacement first, as that is then consumed in odometryTracker
    displacementObserver.update();
    update();
}

}  // namespace aruwsrc::algorithms::odometry
