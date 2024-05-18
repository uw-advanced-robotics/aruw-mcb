#include "chassis_auto_nav_controller.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

namespace aruwsrc::chassis
{
void ChassisAutoNavController::initialize(Position initialPos) {
    path.resetPath();

    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;

    rotateSpeedRamp.reset(chassis.getDesiredRotation());
    xRamp.reset(initialPos.x());
    yRamp.reset(initialPos.y());
}

void ChassisAutoNavController::runController(const uint32_t dt,
                                                const Position currentPos,
                                                const float maxWheelSpeed,
                                                const tap::communication::serial::RefSerialData::Rx::GameType& gametype,
                                                const bool movementEnabled,
                                                const bool beybladeEnabled,
                                                const float chassisYawAngle) {   
    controller_called = true;                      
    Position setPoint = path.setInterpolatedPoint(currentPos);
    float rampTarget = 0.0;
    float x = 0.0f;
    float y = 0.0f;

    if ((int(gametype) == 0 ||
            (drivers.refSerial.getGameData().gameStage == tap::communication::serial::RefSerial::Rx::GameStage::IN_GAME)) &&
        !path.empty() && visionCoprocessor.isCvOnline() && movementEnabled)
    {
        float currentX = setPoint.x();
        float currentY = setPoint.y();

        xRamp.setTarget(currentX);
        yRamp.setTarget(currentY);

        xRamp.update(POS_RAMP_RATE);
        yRamp.update(POS_RAMP_RATE);

        float desiredVelocityX = xRamp.getValue() - currentX;
        float desiredVelocityY = yRamp.getValue() - currentY;
        float mag = sqrtf(pow(desiredVelocityX, 2) + pow(desiredVelocityY, 2));

        if (mag > 0.01)
        {
            x = desiredVelocityX / mag * config.beybladeTranslationalSpeedMultiplier *
                maxWheelSpeed;
            y = desiredVelocityY / mag * config.beybladeTranslationalSpeedMultiplier *
                maxWheelSpeed;
        }
    }

    // float x = xPid.runControllerDerivateError(xRamp.getValue() - currentX, dt) *
    // config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed; float y =
    // yPid.runControllerDerivateError(yRamp.getValue() - currentY, dt) *
    // config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

    if ((int(gametype) == 0 ||
            (drivers.refSerial.getGameData().gameStage == tap::communication::serial::RefSerial::Rx::GameStage::IN_GAME)) &&
        beybladeEnabled)
    {
        // BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE, scaled
        // up by the current max speed, (BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER *
        // maxWheelSpeed)
        const float translationalSpeedThreshold =
            config.translationalSpeedThresholdMultiplierForRotationSpeedDecrease *
            config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

        rampTarget =
            rotationDirection * config.beybladeRotationalSpeedFractionOfMax * maxWheelSpeed;

        // reduce the beyblade rotation when translating to allow for better translational speed
        // (otherwise it is likely that you will barely move unless
        // BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX is small)
        if (fabsf(x) > translationalSpeedThreshold || fabsf(y) > translationalSpeedThreshold)
        {
            rampTarget *= config.beybladeRotationalSpeedMultiplierWhenTranslating;
        }
    }

    rotateSpeedRamp.setTarget(rampTarget);
    // Update the r speed by BEYBLADE_RAMP_UPDATE_RAMP each iteration
    rotateSpeedRamp.update(config.beybladeRampRate);
    float r = rotateSpeedRamp.getValue();

    // Rotate X and Y depending on turret angle
    tap::algorithms::rotateVector(&x, &y, -chassisYawAngle);

    // set outputs
    chassis.setDesiredOutput(x, y, r);

}

} // namespace aruwsrc::chassis
