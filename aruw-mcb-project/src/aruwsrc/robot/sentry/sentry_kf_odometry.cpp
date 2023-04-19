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

#include "aruwsrc/robot/sentry/sentry_kf_odometry.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::algorithms::odometry
{

SentryKFOdometry(
    tap::Drivers& drivers,
    const aruwsrc::control::sentry::drive::HolonomicChassisSubsystem& chassis,
    const aruwsrc::control::turret::SentryTurretMajorSubsystem& turretMajor,
    const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorLeft,
    const aruwsrc::control::turret::SentryTurretMinorSubsystem& turretMinorRight)
    : drivers(drivers), turretMajor(turretMajor), turretMinorLeft(turretMinorLeft), turretMinorRight(turretMinorRight),
    kf(KF_A, KF_C, KF_Q, KF_R, KF_P0)
    { }


SentryKFOdometry::update() {
    // Get chassis positional values
    chassis.get
}

SentryKFOdometry::getCurrentVelocity


} // namespace aruwsrc::algorithms::odometry