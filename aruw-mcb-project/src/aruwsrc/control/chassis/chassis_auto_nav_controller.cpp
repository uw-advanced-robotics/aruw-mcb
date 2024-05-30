#include "chassis_auto_nav_controller.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"

namespace aruwsrc::chassis
{
void ChassisAutoNavController::initialize(Position initialPos)
{
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;

    rotateSpeedRamp.reset(chassis.getDesiredRotation());
    xRamp.reset(initialPos.x());
    yRamp.reset(initialPos.y());

    path.resetPath();
    path.pushPoint(Position(0, 0, 0));
    path.pushPoint(Position(0.75, 0, 0));
    path.pushPoint(Position(0.75, 0.75, 0));
    path.pushPoint(Position(0, 0.75, 0));
    path.pushPoint(Position(0, 0.05, 0));
}

float debugx = 0;
float debugy = 0;
float mag = -1;
Position debugSetPoint = Position(0, 0, 0);
float desiredVelocityX, desiredVelocityY;
float dumbFuckConstant;
void ChassisAutoNavController::runController(
    const uint32_t dt,
    const Position currentPos,
    const float maxWheelSpeed,
    const tap::communication::serial::RefSerialData::Rx::GameType& gametype,
    const bool movementEnabled,
    const bool beybladeEnabled,
    const float chassisYawAngle)
{
    controller_called = true;
    Position setPoint = calculateSetPoint(currentPos, INTERPOLATION_PARAMETER);
    debugSetPoint = setPoint;
    float rampTarget = 0.0;
    float x = 0.0f;
    float y = 0.0f;

    if (((int(gametype) == 0 || (drivers.refSerial.getGameData().gameStage ==
                                 tap::communication::serial::RefSerial::Rx::GameStage::IN_GAME)) &&
         !path.empty() && visionCoprocessor.isCvOnline() && movementEnabled) ||
        true)
    {
        float currentX = currentPos.x();
        float currentY = currentPos.y();

        xRamp.setTarget(setPoint.x());
        yRamp.setTarget(setPoint.y());

        xRamp.update(POS_RAMP_RATE);
        yRamp.update(POS_RAMP_RATE);

        desiredVelocityX = setPoint.x() - currentX;
        desiredVelocityY = setPoint.y() - currentY;
        mag = sqrtf(pow(desiredVelocityX, 2) + pow(desiredVelocityY, 2));

        if (mag > 0.01)
        {
            dumbFuckConstant = config.beybladeTranslationalSpeedMultiplier;
            x = desiredVelocityX / mag * config.beybladeTranslationalSpeedMultiplier *
                maxWheelSpeed;
            y = desiredVelocityY / mag * config.beybladeTranslationalSpeedMultiplier *
                maxWheelSpeed;
        }
    }
    debugx = x;
    debugy = y;

    // float x = xPid.runControllerDerivateError(xRamp.getValue() - currentX, dt) *
    // config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed; float y =
    // yPid.runControllerDerivateError(yRamp.getValue() - currentY, dt) *
    // config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

    // if ((int(gametype) == 0 || (drivers.refSerial.getGameData().gameStage ==
    //                             tap::communication::serial::RefSerial::Rx::GameStage::IN_GAME))
    //                             &&
    //     beybladeEnabled)
    // {
    //     // BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE, scaled
    //     // up by the current max speed, (BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER *
    //     // maxWheelSpeed)
    //     const float translationalSpeedThreshold =
    //         config.translationalSpeedThresholdMultiplierForRotationSpeedDecrease *
    //         config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

    //     rampTarget =
    //         rotationDirection * config.beybladeRotationalSpeedFractionOfMax * maxWheelSpeed;

    //     // reduce the beyblade rotation when translating to allow for better translational speed
    //     // (otherwise it is likely that you will barely move unless
    //     // BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX is small)
    //     if (fabsf(x) > translationalSpeedThreshold || fabsf(y) > translationalSpeedThreshold)
    //     {
    //         rampTarget *= config.beybladeRotationalSpeedMultiplierWhenTranslating;
    //     }
    // }

    // rotateSpeedRamp.setTarget(rampTarget);
    // // Update the r speed by BEYBLADE_RAMP_UPDATE_RAMP each iteration
    // rotateSpeedRamp.update(config.beybladeRampRate);
    // float r = rotateSpeedRamp.getValue();

    // Rotate X and Y depending on turret angle
    tap::algorithms::rotateVector(&x, &y, -chassisYawAngle);

    // set outputs
    chassis.setDesiredOutput(x, y, 0);
}
float closest;
Position ChassisAutoNavController::calculateSetPoint(Position current, float interpolationParameter)
{
    // TODO: account for and deal with the case of a path reset

    if (path.empty())
    {
        return current;
    }

    if (path.hasChanged())
    {
        path.togglePathChanged();
        pathTransitionTimeout.restart(PATH_TRANSITION_TIME_MILLIS);
    }

    closest = path.positionToClosestParameter(current);
    Position lookahead = path.parametertoPosition(closest + interpolationParameter);

    if (!pathTransitionTimeout.isExpired())
        return quadraticBezierInterpolation(
            lookahead,
            current,
            lastSetPoint,
            (float)pathTransitionTimeout.timeRemaining() / PATH_TRANSITION_TIME_MILLIS);

    lastSetPoint = lookahead;
    return lookahead;
}

}  // namespace aruwsrc::chassis
