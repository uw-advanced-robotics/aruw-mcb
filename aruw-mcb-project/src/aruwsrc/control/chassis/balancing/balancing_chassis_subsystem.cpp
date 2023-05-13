/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balancing_chassis_subsystem.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"

namespace aruwsrc::chassis
{
BalancingChassisSubsystem::BalancingChassisSubsystem(
    tap::Drivers* drivers,
    aruwsrc::can::TurretMCBCanComm& turretMCB,
    const aruwsrc::control::turret::TurretMotor& pitchMotor,
    const aruwsrc::control::turret::TurretMotor& yawMotor,
    BalancingLeg& leftLeg,
    BalancingLeg& rightLeg,
    tap::gpio::Analog::Pin currentPin)
    : tap::control::chassis::ChassisSubsystemInterface(drivers),
      rotationPid(AUTOROTATION_PID),
      turretMCB(turretMCB),
      pitchMotor(pitchMotor),
      yawMotor(yawMotor),
      currentSensor(
          {&drivers->analog,
           currentPin,
           aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
           aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
           aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA}),
      chassisPowerLimiter(
          drivers,
          &currentSensor,
          STARTING_ENERGY_BUFFER,
          ENERGY_BUFFER_LIMIT_THRESHOLD,
          ENERGY_BUFFER_CRIT_THRESHOLD),
      leftLeg(leftLeg),
      rightLeg(rightLeg)
{
}

void BalancingChassisSubsystem::initialize()
{
    velocityRamper = tap::algorithms::Ramp();
    desiredX = currentX = 0;
    desiredV = velocityRamper.getValue();
    desiredR = 0;
    desiredZ = leftLeg.getDefaultPosition().getY();
    leftLeg.initialize();
    rightLeg.initialize();
}

void BalancingChassisSubsystem::refresh()
{
    // 1. Update yaw and roll values
    computeState();

    // 2. Apply scaling and/or control laws to yaw and roll values

    // float rollAdjustment = WIDTH_BETWEEN_WHEELS_Y / 2 * sin(roll);
    float rollAdjustment = 0;

    // 3. Set each side's actuators to compensate appropriate for yaw and roll error
    leftLeg.setDesiredHeight(
        tap::algorithms::limitVal<float>(desiredZ + rollAdjustment, -.35, -.15));
    rightLeg.setDesiredHeight(
        tap::algorithms::limitVal<float>(desiredZ - rollAdjustment, -.35, -.15));

    leftLeg.setChassisPos(currentX, desiredX);
    rightLeg.setChassisPos(currentX, desiredX);
    leftLeg.setChassisSpeed(currentV);
    rightLeg.setChassisSpeed(currentV);
    leftLeg.setChassisYaw(desiredR, yawRate);
    // Right leg values are negated due to fun LQR logic
    rightLeg.setChassisYaw(-desiredR, -yawRate);
    leftLeg.setChassisAngle(pitch, pitchRate);
    rightLeg.setChassisAngle(pitch, pitchRate);

    // 4. run outputs
    leftLeg.setDesiredTranslationSpeed(desiredV);  // m/s
    rightLeg.setDesiredTranslationSpeed(desiredV);

    leftLeg.update();
    rightLeg.update();

    /**
     * @warning UN-COMMENT OUT THIS FLAG TO TEST CODE IN SAFE MODE
     */
    // armed = false;

    if (leftLeg.wheelMotorOnline() && rightLeg.wheelMotorOnline() && armed)
    {
        // do this here for safety. Only called once per subsystem. Don't arm leg motors until wheel
        // motors are also online.
        static_cast<aruwsrc::motor::Tmotor_AK809*>(leftLeg.getFiveBar()->getMotor1())
            ->sendCanMessage();
        static_cast<aruwsrc::motor::Tmotor_AK809*>(leftLeg.getFiveBar()->getMotor2())
            ->sendCanMessage();
        static_cast<aruwsrc::motor::Tmotor_AK809*>(rightLeg.getFiveBar()->getMotor1())
            ->sendCanMessage();
        static_cast<aruwsrc::motor::Tmotor_AK809*>(rightLeg.getFiveBar()->getMotor2())
            ->sendCanMessage();
        if (!rightLeg.getArmState()) rightLeg.armLeg();
        if (!leftLeg.getArmState()) leftLeg.armLeg();
    }
    else
    {
        rightLeg.disarmLeg();
        leftLeg.disarmLeg();
    }
}

void BalancingChassisSubsystem::getAngles(uint32_t dt)
{
    // Value from [0, 2Pi]
    float currentTurretPitch = pitchMotor.getChassisFrameMeasuredAngle().getValue() -
                               aruwsrc::control::turret::PITCH_MOTOR_CONFIG.startAngle -
                               aruwsrc::control::turret::CHASSIS_FALL_OVER_OFFSET;
    if (currentTurretPitch > M_PI) currentTurretPitch -= M_TWOPI;
    float currentTurretYaw = yawMotor.getChassisFrameMeasuredAngle().getValue();
    if (currentTurretYaw > M_PI) currentTurretYaw -= M_TWOPI;

    float worldRelativeTurretPitch = -turretMCB.getPitch() + M_PI;
    if (worldRelativeTurretPitch > M_PI) worldRelativeTurretPitch -= M_TWOPI;
    float worldRelativeTurretRoll = -turretMCB.getRoll() + M_PI;
    if (worldRelativeTurretRoll > M_PI) worldRelativeTurretRoll -= M_TWOPI;
    float worldRelativeTurretYaw = turretMCB.getYaw();

    debug1 = worldRelativeTurretPitch;
    debug2 = worldRelativeTurretRoll;
    debug3 = worldRelativeTurretYaw;
    debug4 = currentTurretPitch;

    tap::algorithms::transforms::Transform<World, Turret> worldToTurret =
        tap::algorithms::transforms::Transform<World, Turret>(
            0,
            0,
            0,
            worldRelativeTurretRoll,
            worldRelativeTurretPitch,
            worldRelativeTurretYaw);

    // Define the transformation from Turret to Chassis based on turret encoders
    chassisToTurret.updateRotation(0, currentTurretPitch, currentTurretYaw);
    tap::algorithms::transforms::Transform<World, Chassis> worldToChassis =
        tap::algorithms::transforms::compose(worldToTurret, chassisToTurret.getInverse());

    // Define the current Turret Orientations in world-frame.
    // CMSISMat<3, 1> worldRelativeTurretOrientation = {
    //     {worldRelativeTurretRoll, worldRelativeTurretPitch, worldRelativeTurretYaw}};
    // CMSISMat<3, 1> chassisRelativeTurretOrientation = {{0, currentTurretPitch,
    // currentTurretYaw}};
    // CMSISMat<3, 1> worldRelativeTurretOrientationRate = {
    //     {-turretMCB.getRollVelocity(), -turretMCB.getPitchVelocity(), turretMCB.getYawVelocity()}};
    // CMSISMat<3, 1> chassisRelativeTurretOrientationRate = {
    //     {0, pitchMotor.getChassisFrameVelocity(), yawMotor.getChassisFrameVelocity()}};

    roll = worldToChassis.getRoll();
    pitch = worldToChassis.getPitch();
    yaw = worldToChassis.getYaw();

    // CMSISMat<3, 1> worldRelativeChassisOrientationRate =
    //     chassisToTurret.applyToVector(worldRelativeTurretOrientationRate) -
    //     chassisRelativeTurretOrientationRate;
    // rollRate = worldRelativeChassisOrientationRate.data[0];
    // pitchRate = worldRelativeChassisOrientationRate.data[1];
    // yawRate = worldRelativeChassisOrientationRate.data[2];

    float pitchRateNew = (pitch - pitchPrev) * 1'000 / dt;
    pitchPrev = pitch;
    // pitchRate = lowPassFilter(
    //     pitchRate,
    //     -turretMCB.getPitchVelocity() - pitchMotor.getChassisFrameVelocity(),
    //     .05);
    pitchRate = lowPassFilter(pitchRate, pitchRateNew, .05);

    float yawRateNew = (yaw - yawPrev) * 1000.0f / static_cast<float>(dt);
    // yawRate = turretMCB.getYawVelocity() - yawMotor.getChassisFrameVelocity();
    yawPrev = yaw;
    yawRate = lowPassFilter(yawRate, yawRateNew, .05);
}

void BalancingChassisSubsystem::computeState()
{
    uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    getAngles(dt);

    velocityRamper.update(dt / 1000 * MAX_ACCELERATION);
    desiredV = velocityRamper.getTarget();

    float newCurrentV =
        (rightLeg.getCurrentTranslationSpeed() + leftLeg.getCurrentTranslationSpeed()) / 2;
    currentV = lowPassFilter(currentV, newCurrentV, 1);

    float rightRot =
        2 * (rightLeg.getCurrentTranslationSpeed() - currentV) / WIDTH_BETWEEN_WHEELS_Y;
    float leftRot = 2 * (leftLeg.getCurrentTranslationSpeed() + currentV) / WIDTH_BETWEEN_WHEELS_Y;
    currentR = (rightRot + leftRot) / 2;
    currentZ = rightLeg.getCurrentHeight() > leftLeg.getCurrentHeight()
                   ? rightLeg.getCurrentHeight()
                   : leftLeg.getCurrentHeight();

    currentX = (leftLeg.getWheelPos() + rightLeg.getWheelPos()) / 2;
    if (desiredV == 0 && prevVdesired != 0)
    {
        desiredX = currentX;
    }
    else if (desiredV != 0)
    {
        desiredX += desiredV / WHEEL_RADIUS * dt / 1'000'000;
    }
    prevVdesired = desiredV;
}

void BalancingChassisSubsystem::runHardwareTests() {}
}  // namespace aruwsrc::chassis
