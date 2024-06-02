#include "chassis_auto_nav_controller.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"

namespace aruwsrc::chassis
{
void ChassisAutoNavController::initialize()
{
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;

    lastSetPoint = Position(worldToChassis.getX(), worldToChassis.getY(), 0);
    rotateSpeedRamp.reset(chassis.getDesiredRotation());

    path.resetPath();
    path.pushPoint(Position(0.0, 0.0, 0));  // sorry about this lol
    path.pushPoint(Position(0.1, 0.0, 0));  // somehow this felt like less effort than using loops..
    path.pushPoint(Position(0.2, 0.0, 0));
    path.pushPoint(Position(0.3, 0.0, 0));
    path.pushPoint(Position(0.4, 0.0, 0));
    path.pushPoint(Position(0.5, 0.0, 0));
    path.pushPoint(Position(0.6, 0.0, 0));
    path.pushPoint(Position(0.7, 0.0, 0));
    path.pushPoint(Position(0.7, 0.0, 0));
    path.pushPoint(Position(0.7, 0.1, 0));
    path.pushPoint(Position(0.7, 0.2, 0));
    path.pushPoint(Position(0.7, 0.3, 0));
    path.pushPoint(Position(0.7, 0.4, 0));
    path.pushPoint(Position(0.7, 0.5, 0));
    path.pushPoint(Position(0.7, 0.6, 0));
    path.pushPoint(Position(0.7, 0.7, 0));
    path.pushPoint(Position(0.6, 0.7, 0));
    path.pushPoint(Position(0.5, 0.7, 0));
    path.pushPoint(Position(0.4, 0.7, 0));
    path.pushPoint(Position(0.3, 0.7, 0));
    path.pushPoint(Position(0.2, 0.7, 0));
    path.pushPoint(Position(0.1, 0.7, 0));
    path.pushPoint(Position(0.0, 0.7, 0));
    path.pushPoint(Position(0.0, 0.6, 0));
    path.pushPoint(Position(0.0, 0.5, 0));
    path.pushPoint(Position(0.0, 0.4, 0));
    path.pushPoint(Position(0.0, 0.3, 0));
    path.pushPoint(Position(0.0, 0.2, 0));
    // path.pushPoint(Position(0.0, 0.1, 0));
    // path.pushPoint(Position(0.0, 0.0, 0));
}

// debug declarations
float closest;
float mag = -1;
Position currentPos = Position(0, 0, 0);
Position setpoint = Position(0, 0, 0);
Vector moveVector(-1, -1, 0);
Vector chassisFrameMoveVector(0, 0, 0);

void ChassisAutoNavController::runController(
    const float maxWheelSpeed,
    const bool movementEnabled,
    const bool beybladeEnabled)
{
    currentPos = worldToChassis.getTranslation();  // works bc transformer always makes z 0
    float lookaheadDist = LOOKAHEAD_DISTANCE;  // redeclared here bc it might be useful to replace
                                               // this constant with a function in the future
    setpoint = calculateSetPoint(currentPos, lookaheadDist, movementEnabled);
    float rampTarget = 0.0;

    moveVector = Vector(0, 0, 0);

    Vector posError = setpoint - currentPos;
    mag = posError.magnitude();

    float desiredSpeed = visionCoprocessor.getAutonavSpeed();

    if (mag > 0.01)
    {
        moveVector = posError / lookaheadDist * desiredSpeed * chassis.mpsToRpm(1);
    }

    // float x = xPid.runControllerDerivateError(xRamp.getValue() - currentX, dt) *
    // config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed; float y =
    // yPid.runControllerDerivateError(yRamp.getValue() - currentY, dt) *
    // config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

    // BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE, scaled
    // up by the current max speed, (BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER *
    // maxWheelSpeed)
    const float translationalSpeedThreshold =
        config.translationalSpeedThresholdMultiplierForRotationSpeedDecrease *
        config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

    rampTarget = rotationDirection * config.beybladeRotationalSpeedFractionOfMax * maxWheelSpeed;

    // reduce the beyblade rotation when translating to allow for better translational speed
    // (otherwise it is likely that you will barely move unless
    // BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX is small)
    if (moveVector.magnitude() > translationalSpeedThreshold)
    {
        rampTarget *= config.beybladeRotationalSpeedMultiplierWhenTranslating;
    }

    rotateSpeedRamp.setTarget(rampTarget);
    // Update the r speed by BEYBLADE_RAMP_UPDATE_RAMP each iteration
    rotateSpeedRamp.update(config.beybladeRampRate);
    float r = rotateSpeedRamp.getValue();

    // convert world frame translation to chassis frame
    chassisFrameMoveVector = worldToChassis.apply(moveVector);

    // set outputs
    chassis.setDesiredOutput(
        chassisFrameMoveVector.x(),
        chassisFrameMoveVector.y(),
        beybladeEnabled ? r : 0);
}

Position ChassisAutoNavController::calculateSetPoint(
    Position current,
    float lookaheadDistance,
    bool movementEnabled)
{
    if (!visionCoprocessor.isCvOnline() || !movementEnabled)
    {
        return lastSetPoint;
    }

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

    Position lookaheadPos = path.parametertoPosition(closest + lookaheadDistance);

    if (!pathTransitionTimeout.isExpired())
        return quadraticBezierInterpolation(
            lookaheadPos,
            current,
            lastSetPoint,
            (float)pathTransitionTimeout.timeRemaining() / PATH_TRANSITION_TIME_MILLIS);

    lastSetPoint = lookaheadPos;
    return lookaheadPos;
}

}  // namespace aruwsrc::chassis
