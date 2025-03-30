/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruco_reset_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::aruco
{
ArucoResetSubsystem::ArucoResetSubsystem(
    tap::Drivers* drivers,
    VisionCoprocessor& vision,
    DeadwheelChassisKFOdometry& odometry,
    TransformerInterface& transformer)
    : tap::control::Subsystem(drivers),
      vision(vision),
      odometry(odometry),
      transformer(transformer)
{
}

void ArucoResetSubsystem::refresh()
{
    const VisionCoprocessor::ArucoResetData& resetData = vision.getLastArucoResetData();

    if (!resetData.updated) return;
    vision.invalidateArucoResetData();

    float prevComputedX = odometry.getCurrentLocation2D().getX();
    float prevComputedY = odometry.getCurrentLocation2D().getY();

    // Get the chassis position estimate from the aruco data
    float arucoChassisXEstimate = resetData.data.x -
                                  transformer.getWorldToTurret(resetData.data.turretId).getX() +
                                  transformer.getWorldToChassis().getX();
    float arucoChassisYEstimate = resetData.data.y -
                                  transformer.getWorldToTurret(resetData.data.turretId).getY() +
                                  transformer.getWorldToChassis().getY();

    // Apply a low-pass between the aruco measurement and our current odometry position
    float newX = lowPassFilter(prevComputedX, arucoChassisXEstimate, VISION_TRUST);
    float newY = lowPassFilter(prevComputedY, arucoChassisYEstimate, VISION_TRUST);

    // Set the new position in the odometry subsystem
    odometry.overrideOdometryPosition(modm::Vector2f(newX, newY));
}

}  // namespace aruwsrc::control::aruco
