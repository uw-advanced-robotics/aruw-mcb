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

#include "sentinel_otto_kf_odometry_2d_subsystem.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::algorithms::odometry
{
SentinelOttoKFOdometry2DSubsystem::SentinelOttoKFOdometry2DSubsystem(
    aruwsrc::Drivers& drivers,
    const aruwsrc::control::sentinel::drive::SentinelDriveSubsystem& chassis,
    const aruwsrc::control::turret::TurretSubsystem& turret)
    : Subsystem(&drivers),
      SentinelChassisKFOdometry(drivers, chassis, turret)
{
}

void SentinelOttoKFOdometry2DSubsystem::refresh() { update(); }

}  // namespace aruwsrc::algorithms::odometry
