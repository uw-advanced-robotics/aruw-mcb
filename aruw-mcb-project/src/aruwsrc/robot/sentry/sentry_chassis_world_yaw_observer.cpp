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

#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::sentry
{
SentryChassisWorldYawObserver::SentryChassisWorldYawObserver(
    tap::communication::sensors::imu::ImuInterface& imu,
    const aruwsrc::control::turret::YawTurretSubsystem& turretMajor)
    : imu(imu),
      turretMajor(turretMajor)
{
}

bool SentryChassisWorldYawObserver::getChassisWorldYaw(float* output) const
{
    // TODO: Make this false while the IMU is uncalibrated. Not possible via Interface
    float turretMajorChassisYawRadians = turretMajor.getReadOnlyMotor().getAngleFromCenter();
    *output = modm::Angle::normalize(
        modm::toRadian(imu.getYaw()) + offset - turretMajorChassisYawRadians);
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
