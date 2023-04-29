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

namespace aruwsrc::chassis
{
BalancingChassisSubsystem::BalancingChassisSubsystem(
    tap::Drivers* drivers,
    aruwsrc::can::TurretMCBCanComm& turretMCB,
    BalancingLeg& leftLeg,
    BalancingLeg& rightLeg,
    tap::gpio::Analog::Pin currentPin)
    : Subsystem(drivers),
      rotationPid(AUTOROTATION_PID),
      turretMCB(turretMCB),
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

    // 4. run outputs
    leftLeg.setDesiredTranslationSpeed(desiredV);  // m/s
    rightLeg.setDesiredTranslationSpeed(desiredV);

    leftLeg.update();
    rightLeg.update();
    // do this here for safety. Only called once per subsystem. Don't arm leg motors until wheel
    // motors are also online.
    if (leftLeg.wheelMotorOnline() && rightLeg.wheelMotorOnline())
    {
        static_cast<aruwsrc::motor::Tmotor_AK809*>(leftLeg.getFiveBar()->getMotor1())
            ->sendCanMessage();
        static_cast<aruwsrc::motor::Tmotor_AK809*>(leftLeg.getFiveBar()->getMotor2())
            ->sendCanMessage();
        static_cast<aruwsrc::motor::Tmotor_AK809*>(rightLeg.getFiveBar()->getMotor1())
            ->sendCanMessage();
        static_cast<aruwsrc::motor::Tmotor_AK809*>(rightLeg.getFiveBar()->getMotor2())
            ->sendCanMessage();
    }
}

void BalancingChassisSubsystem::computeState()
{
    uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = curTime - prevTime;
    prevTime = curTime;

    pitch = turretMCB.getPitch();
    roll = drivers->mpu6500.getPitch() * M_TWOPI / 360;
    yaw = turretMCB.getYaw();

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

    yawRate = (yaw - yawPrev) * 1000.0f / static_cast<float>(dt);
    yawPrev = yaw;

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
