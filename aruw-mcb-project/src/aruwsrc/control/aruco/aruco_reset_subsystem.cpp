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
    Odometry2DInterface& odometry,
    TransformerInterface& transformer)
    : tap::control::Subsystem(drivers),
      vision(vision),
      odometry(odometry),
      transformer(transformer)
{
}

void ArucoResetSubsystem::refresh()
{
    processRealsenseData();
    processArducamData();
}

void ArucoResetSubsystem::processRealsenseData()
{
    const VisionCoprocessor::ArucoResetData& resetData = vision.getLastRealsenseArucoData();
    if (!resetData.updated) return;
    vision.invalidateRealsenseArucoResetData();

    // Get the chassis position estimate from the aruco data
    float arucoChassisXEstimate = resetData.data.x - // world To turret
                                  transformer.getWorldToTurret(resetData.data.turretId).getX() +
                                  transformer.getWorldToChassis().getX();

    float arucoChassisYEstimate = resetData.data.y -
                                  transformer.getWorldToTurret(resetData.data.turretId).getY() +
                                  transformer.getWorldToChassis().getY();

    // Set the new position in the odometry subsystem
    odometry.overrideOdometryPosition(arucoChassisXEstimate, arucoChassisYEstimate);
}

EulerAngles angles;
Transform worldToCamera(Transform::identity()), cameraToChassis(Transform::identity()),
    worldToChassis(Transform::identity());
void ArucoResetSubsystem::processArducamData()
{
    const VisionCoprocessor::ArucoResetData& resetData = vision.getLastArducamArucoData();
    if (!resetData.updated) return;
    vision.invalidateArducamArucoResetData();

    const VisionCoprocessor::ArucoResetPacket& poseData = resetData.data;

    angles =
        quaternionToEulerAngles(poseData.quatW, poseData.quatX, poseData.quatY, poseData.quatZ);

    worldToCamera =
        Transform(poseData.x, poseData.y, poseData.z, angles.roll, angles.pitch, angles.yaw);
    cameraToChassis = transformer.getChassisToArducam(resetData.data.turretId).getInverse();

    worldToChassis = worldToCamera.compose(cameraToChassis);

    float newX = worldToChassis.getX();
    float newY = worldToChassis.getY();

    float prevX = odometry.getCurrentLocation2D().getX();
    float prevY = odometry.getCurrentLocation2D().getY();

    // Apply a low-pass between the aruco measurement and our current odometry position
    newX = lowPassFilter(prevX, newX, VISION_TRUST);
    newY = lowPassFilter(prevY, newY, VISION_TRUST);

    odometry.overrideOdometryPosition(newX, newY);

    // Get the chassis position estimate from the aruco data
    // float arucoChassisXEstimate =
    //     resetData.data.x +
    //     transformer.getChassisToArducam(resetData.data.turretId).getInverse().getX();

    // float arucoChassisYEstimate =
    //     resetData.data.y +
    //     transformer.getChassisToArducam(resetData.data.turretId).getInverse().getY();

    // Apply a low-pass between the aruco measurement and our current odometry position
    // float newX = lowPassFilter(prevX, arucoChassisXEstimate, VISION_TRUST);
    // float newY = lowPassFilter(prevY, arucoChassisYEstimate, VISION_TRUST);

    // // Set the new position in the odometry subsystem
    // odometry.overrideOdometryPosition(newX, newY);
}

EulerAngles quaternionToEulerAngles(float w, float x, float y, float z)
{
    EulerAngles angles;
    angles.roll = atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    angles.pitch = asin(2.0f * (w * y - z * x));
    angles.yaw = atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    return angles;
}

}  // namespace aruwsrc::control::aruco
