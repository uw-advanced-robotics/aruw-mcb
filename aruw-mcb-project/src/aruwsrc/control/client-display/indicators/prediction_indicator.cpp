/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "prediction_indicator.hpp"

using namespace tap::communication::serial;
using namespace tap::algorithms::ballistics;

namespace aruwsrc::control::client_display::indicators
{
PredictionIndicator::PredictionIndicator(
    aruwsrc::communication::serial::VisionCoprocessor &visionCoprocessor,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const control::turret::RobotTurretSubsystem &turretSubsystem,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const float defaultLaunchSpeed,
    const Transform &worldToTurret)
    : HudIndicator(refSerialTransmitter),
      visionCoprocessor(visionCoprocessor),
      refSerialTransmitter(refSerialTransmitter),
      odometryInterface(odometryInterface),
      turretSubsystem(turretSubsystem),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed),
      worldToTurret(worldToTurret)
{
}

modm::ResumableResult<void> PredictionIndicator::update()
{
    const float plateHeight = 0.15f;  // TODO don't hardcode thus
    modm::Vector3f predictedShotLandingPosition;
    float time;
    ProjectedResult result;

    // if the friction wheel launch speed is 0, use a default launch speed so ballistics
    // gives a reasonable computation
    float launchSpeed = frictionWheels.getPredictedLaunchSpeed();

    if (compareFloatClose(launchSpeed, 0.0f, 1e-5f))
    {
        launchSpeed = defaultLaunchSpeed;
    }

    // defines the turret where the chassis is, under the assumption that the chassis origin and
    // turret origin coincide
    modm::Vector3f turretPosition(odometryInterface.getCurrentLocation2D().getPosition(), 0);
    modm::Vector3f turretRotation(
        turretSubsystem.getWorldYaw(),
        turretSubsystem.getWorldPitch(),
        0);

    // Puts turret in it's place in world frame
    // If no offset, skip all offsetting
    if (turretSubsystem.getTurretOffset() != modm::Vector3f(0, 0, 0))
    {
        // make this in here to minimize resource usage I guess
        modm::Vector3f turretOffset = turretSubsystem.getTurretOffset();
        // yaw is 0, so chassis frame and world frame share orientation. They may not share
        // translation, so we still need to add that.
        if (compareFloatClose(odometryInterface.getYaw(), 0.0f, 1e-5f))
        {
            // Assume that z is parallel to yaw and needs not adjusting.
            // This breaks if the robot rolls, but we'd need to implement 3D odometry anyways
            // soooo not my problem! For now, skips 3D vector rotation.
            rotateVector(&turretOffset.x, &turretOffset.y, odometryInterface.getYaw());
        }
        turretPosition += turretOffset;
    }

    ballistics::SecondOrderKinematicState predictedShotLandingState(
        turretPosition,
        turretRotation * launchSpeed,
        modm::Vector3f(0, -tap::algorithms::ACCELERATION_GRAVITY, 0));
    RF_BEGIN(1);

    // calculate the time it would take for the shot to reach the plate height
    time = (-predictedShotLandingState.velocity.z -
            sqrtf(
                powf(predictedShotLandingState.velocity.z, 2) -
                2 * tap::algorithms::ACCELERATION_GRAVITY *
                    (predictedShotLandingState.position.z - plateHeight))) /
           tap::algorithms::ACCELERATION_GRAVITY;

    // calculate the position of the shot when it reaches the plate height
    predictedShotLandingPosition = predictedShotLandingState.projectForward(time);

    // project the predicted shot landing position into the camera frame and then to screen
    // coordinates
    result = convertCameraFrameToScreenFrame(Position(
        predictedShotLandingPosition.getX(),
        predictedShotLandingPosition.getY(),
        predictedShotLandingPosition.getZ()));

    // If the predicted landing position is not in frame, delete the graphic
    if (!result.inFrame)
    {
        hitPredictionGraphic.graphicData.operation = Tx::GRAPHIC_DELETE;
    }
    else
    {
        hitPredictionGraphic.graphicData.operation =
            hitPredictionGraphic.graphicData.operation == Tx::GRAPHIC_DELETE ? Tx::GRAPHIC_ADD
                                                                             : Tx::GRAPHIC_MODIFY;
    }

    // Send the graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&hitPredictionGraphic, true, true, false));
    RF_END();
}
}  // namespace aruwsrc::control::client_display::indicators