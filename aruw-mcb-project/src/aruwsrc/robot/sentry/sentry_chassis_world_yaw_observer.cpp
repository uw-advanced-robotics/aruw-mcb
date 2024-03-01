/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "sentry_chassis_world_yaw_observer.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/util_macros.hpp"
#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::sentry
{
SentryChassisWorldYawObserver::SentryChassisWorldYawObserver(
    aruwsrc::control::turret::YawTurretSubsystem& turretMajor,
    const aruwsrc::control::turret::TurretSubsystem& turretLeft,
    const aruwsrc::control::turret::TurretSubsystem& turretRight)
    : turretMajor(turretMajor),
      turretLeft(turretLeft),
      turretRight(turretRight)
{
}

bool SentryChassisWorldYawObserver::getChassisWorldYaw(float* output) const
{
    auto turretMCB = turretRight.getTurretMCB();
    assert(turretMCB != nullptr);

    auto& turretMajorMotor = turretMajor.getReadOnlyMotor();

    if (!turretMCB->isConnected() || !turretRight.yawMotor.isOnline() ||
        !turretMajorMotor.isOnline())
    {
        return false;
    }
    float turretWorldYawRadians = modm::Angle::normalize(turretMCB->getYaw());
    float turretMinorMajorYawRadians = turretRight.yawMotor.getAngleFromCenter();
    float turretMajorChassisYawRadians = turretMajorMotor.getAngleFromCenter();

    *output = modm::Angle::normalize(
        turretWorldYawRadians - turretMinorMajorYawRadians - turretMajorChassisYawRadians + offset);
    return true;
}

void SentryChassisWorldYawObserver::overrideChassisYaw(float newYaw)
{
    float currYaw;
    if (getChassisWorldYaw(&currYaw))
    {
        offset += newYaw - currYaw;  // no need to normalize, done in getChassisWorldYaw
    }
}

}  // namespace aruwsrc::sentry
