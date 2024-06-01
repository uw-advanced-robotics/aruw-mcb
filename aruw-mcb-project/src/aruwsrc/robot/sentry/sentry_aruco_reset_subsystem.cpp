/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_aruco_reset_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "modm/math/geometry/quaternion.hpp"
#include "modm/math/geometry/vector2.hpp"

using namespace tap::algorithms::transforms;
using namespace aruwsrc::sentry;

SentryArucoResetSubsystem::SentryArucoResetSubsystem(
    tap::Drivers& drivers,
    aruwsrc::serial::VisionCoprocessor& vision,
    aruwsrc::sentry::SentryChassisWorldYawObserver& yawObserver,
    aruwsrc::sentry::SentryKFOdometry2DSubsystem& odometrySubsystem,
    SentryTransforms& transforms)
    : tap::control::Subsystem(&drivers),
      vision(vision),
      yawObserver(yawObserver),
      odometrySubsystem(odometrySubsystem),
      transforms(transforms)
{
}

void SentryArucoResetSubsystem::refresh()
{
    const aruwsrc::serial::VisionCoprocessor::ArucoResetData& resetData =
        vision.getLastArucoResetData();

    if (!resetData.updated) return;
    vision.invalidateArucoResetData();

    modm::Quaternion<float> q(
        resetData.data.quatW,
        resetData.data.quatX,
        resetData.data.quatY,
        resetData.data.quatZ);

    const Transform& majorToMinor = transforms.getMajorToMinor(resetData.data.turretId);
    const Transform& chassisToMajor = transforms.getChassisToMajor();

    float newYaw = tap::algorithms::eulerAnglesFromQuaternion(q).z - majorToMinor.getYaw() -
                   chassisToMajor.getYaw();

    float oldYaw;
    yawObserver.getChassisWorldYaw(&oldYaw);

    float chassisX = resetData.data.x -
                     transforms.getWorldToTurret(resetData.data.turretId).getX() +
                     transforms.getWorldToChassis().getX();
    float chassisY = resetData.data.y -
                     transforms.getWorldToTurret(resetData.data.turretId).getY() +
                     transforms.getWorldToChassis().getY();

    setOrientation(newYaw, oldYaw);
    setPosition(chassisX, chassisY);
}

void SentryArucoResetSubsystem::setOrientation(float newYaw, float oldYaw)
{
    odometrySubsystem.overrideOdometryOrientation(newYaw - oldYaw);
    yawObserver.overrideChassisYaw(newYaw);
}

void SentryArucoResetSubsystem::setPosition(const float x, const float y)
{
    odometrySubsystem.overrideOdometryPosition(modm::Vector2f(x, y));
}
