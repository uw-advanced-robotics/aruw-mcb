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

#include <algorithm>
#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::aruco
{
ArucoResetSubsystem::ArucoResetSubsystem(
    tap::Drivers* drivers,
    VisionCoprocessor& vision,
    Odometry2DInterface& odometry,
    TransformerInterface& transformer,
    FourWheelEKFOdometry* wheelEkfOdometry)
    : tap::control::Subsystem(drivers),
      vision(vision),
      odometry(odometry),
      transformer(transformer),
      wheelEkfOdometry(wheelEkfOdometry)
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
    float arucoChassisXEstimate = resetData.data.x -
                                  transformer.getWorldToTurret(resetData.data.turretId).getX() +
                                  transformer.getWorldToChassis().getX();

    float arucoChassisYEstimate = resetData.data.y -
                                  transformer.getWorldToTurret(resetData.data.turretId).getY() +
                                  transformer.getWorldToChassis().getY();

    fuseVisionPositionMeasurement(
        modm::Vector2f(arucoChassisXEstimate, arucoChassisYEstimate),
        calculateRealsensePositionVariance(),
        calculateRealsensePositionVariance());
}

void ArucoResetSubsystem::processArducamData()
{
    const VisionCoprocessor::ArucoResetData& resetData = vision.getLastArducamArucoData();
    if (!resetData.updated) return;
    vision.invalidateArducamArucoResetData();

    const VisionCoprocessor::ArucoResetPacket& poseData = resetData.data;

    modm::Quaternion q(poseData.quatW, poseData.quatX, poseData.quatY, poseData.quatZ);
    modm::Vector3f angles = eulerAnglesFromQuaternion(q);

    Transform worldToCamera =
        Transform(poseData.x, poseData.y, poseData.z, angles.x, angles.y, angles.z);
    Transform cameraToChassis =
        transformer.getChassisToArducam(resetData.data.turretId).getInverse();

    Transform worldToChassis = worldToCamera.compose(cameraToChassis);

    const modm::Vector2f measuredPosition(worldToChassis.getX(), worldToChassis.getY());
    const float positionVariance = calculateArducamPositionVariance(poseData);
    const float yawVariance = calculateArducamYawVariance(poseData);

    if (!hasReceivedVisionMeasurement)
    {
        if (wheelEkfOdometry != nullptr)
        {
            wheelEkfOdometry->initializeVisionPose(
                measuredPosition,
                worldToChassis.getYaw(),
                positionVariance,
                positionVariance,
                yawVariance);
        }
        else
        {
            odometry.overrideOdometryPosition(measuredPosition.x, measuredPosition.y);
        }
        hasReceivedVisionMeasurement = true;
        return;
    }

    if (wheelEkfOdometry != nullptr)
    {
        FourWheelEKFOdometry::VisionPoseMeasurement visionMeasurement{};
        visionMeasurement.position = measuredPosition;
        visionMeasurement.positionVarianceX = positionVariance;
        visionMeasurement.positionVarianceY = positionVariance;
        visionMeasurement.source = FourWheelEKFOdometry::VisionMeasurementSource::APRIL_TAG;
        visionMeasurement.yaw = worldToChassis.getYaw();
        visionMeasurement.yawVariance = yawVariance;
        // wheelEkfOdometry->fuseVisionPose(visionMeasurement);
        wheelEkfOdometry->fuseVisionPosition(visionMeasurement);
        return;
    }

    fuseVisionPositionMeasurement(measuredPosition, positionVariance, positionVariance);
}

void ArucoResetSubsystem::fuseVisionPositionMeasurement(
    const modm::Vector2f& measuredPosition,
    float positionVarianceX,
    float positionVarianceY)
{
    if (!hasReceivedVisionMeasurement)
    {
        initializeVisionPositionMeasurement(measuredPosition, positionVarianceX, positionVarianceY);
        hasReceivedVisionMeasurement = true;
        return;
    }

    if (wheelEkfOdometry != nullptr)
    {
        FourWheelEKFOdometry::VisionPositionMeasurement visionMeasurement{
            measuredPosition,
            positionVarianceX,
            positionVarianceY,
            FourWheelEKFOdometry::VisionMeasurementSource::GENERIC,
        };
        wheelEkfOdometry->fuseVisionPosition(visionMeasurement);
        return;
    }

    float newX =
        lowPassFilter(odometry.getCurrentLocation2D().getX(), measuredPosition.x, VISION_TRUST);
    float newY =
        lowPassFilter(odometry.getCurrentLocation2D().getY(), measuredPosition.y, VISION_TRUST);
    odometry.overrideOdometryPosition(newX, newY);
}

void ArucoResetSubsystem::initializeVisionPositionMeasurement(
    const modm::Vector2f& measuredPosition,
    float positionVarianceX,
    float positionVarianceY)
{
    if (wheelEkfOdometry != nullptr)
    {
        wheelEkfOdometry->initializeVisionPosition(
            measuredPosition,
            positionVarianceX,
            positionVarianceY);
        return;
    }

    odometry.overrideOdometryPosition(measuredPosition.x, measuredPosition.y);
}

float ArucoResetSubsystem::calculateArducamPositionVariance(
    const VisionCoprocessor::ArucoResetPacket& poseData) const
{
    static constexpr float MIN_POSITION_VARIANCE = 1.0e-6f;
    static constexpr float DISTANCE_VARIANCE_SCALE = 0.004f;
    // static constexpr float ANGLE_VARIANCE_SCALE = 0.000001f;

    const float distance = std::max(0.0f, poseData.cameraToTagMagnitude);
    const float angle = std::abs(poseData.cameraToTagAngle);
    const float standardDeviation =
        DISTANCE_VARIANCE_SCALE * distance; /*+ ANGLE_VARIANCE_SCALE * distance * angle;*/

    return std::max(MIN_POSITION_VARIANCE, standardDeviation * standardDeviation);
}

float ArucoResetSubsystem::calculateArducamYawVariance(
    const VisionCoprocessor::ArucoResetPacket& poseData) const
{
    static constexpr float MIN_YAW_VARIANCE = 1.0e-6f;
    static constexpr float DISTANCE_YAW_VARIANCE_SCALE = 0.002f;
    static constexpr float ANGLE_YAW_VARIANCE_SCALE = 0.008f;

    const float distance = std::max(0.0f, poseData.cameraToTagMagnitude);
    const float angle = std::abs(poseData.cameraToTagAngle);
    const float standardDeviation =
        DISTANCE_YAW_VARIANCE_SCALE * distance + ANGLE_YAW_VARIANCE_SCALE * angle;

    return std::max(MIN_YAW_VARIANCE, standardDeviation * standardDeviation);
}

float ArucoResetSubsystem::calculateRealsensePositionVariance() const
{
    static constexpr float REALSENSE_POSITION_VARIANCE = 1.0e-4f;
    return REALSENSE_POSITION_VARIANCE;
}

}  // namespace aruwsrc::control::aruco
