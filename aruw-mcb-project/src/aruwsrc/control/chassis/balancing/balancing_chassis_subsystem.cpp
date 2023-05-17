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
    uint32_t curTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = curTime - prevTime;
    prevTime = curTime;
    // 1. Update yaw and roll values
    computeState(dt);

    // 2. Apply scaling and/or control laws to yaw and roll values
    updateLegOdometry();
    // 3. Set each side's actuators to compensate appropriate for yaw and roll error

    // 4. run outputs
    float rollAdjustment = WIDTH_BETWEEN_WHEELS_Y / 2 * sin(roll);

    leftLeg.setDesiredHeight(
        tap::algorithms::limitVal<float>(desiredZ + rollAdjustment, -.35, -.15));
    rightLeg.setDesiredHeight(
        tap::algorithms::limitVal<float>(desiredZ - rollAdjustment, -.35, -.15));
    leftLeg.setDesiredTranslationSpeed(desiredV);  // m/s
    rightLeg.setDesiredTranslationSpeed(desiredV);

    if (homingState == HOMING)
    {
        // Call this below
        homeLegs(dt);
        // Call this above updating the legs to ensure that the armed flag is false, therefore
        // sending no wheel output. This also ensure the fivebar controllers don't set output
    }
    // Call this here to pass the chassis arm state to the legs, disabling wheel motor motion.
    !armed ? disarmChassis() : armChassis();
    leftLeg.update();
    rightLeg.update();

    if (leftLeg.wheelMotorOnline() && rightLeg.wheelMotorOnline())
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

    roll = worldToChassis.getRoll();
    pitch = worldToChassis.getPitch();
    yaw = worldToChassis.getYaw();

    float pitchRateNew = (pitch - pitchPrev) * 1'000 / dt;
    pitchPrev = pitch;
    pitchRate = lowPassFilter(pitchRate, pitchRateNew, .05);

    float yawRateNew = (yaw - yawPrev) * 1000.0f / static_cast<float>(dt);
    yawPrev = yaw;
    yawRate = lowPassFilter(yawRate, yawRateNew, .05);
}

void BalancingChassisSubsystem::computeState(uint32_t dt)
{
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

void BalancingChassisSubsystem::homeLegs(uint32_t dt)
{
    if (homingState != HOMING) return;
    // 1. PID legs to an angle slightly more towards their home until they reach it.
    // Fronts want to rotate negative
    static aruwsrc::motor::Tmotor_AK809* legMotors[4] = {
        static_cast<aruwsrc::motor::Tmotor_AK809*>(getLeftLeg().getFiveBar()->getMotor1()),
        static_cast<aruwsrc::motor::Tmotor_AK809*>(getLeftLeg().getFiveBar()->getMotor2()),
        static_cast<aruwsrc::motor::Tmotor_AK809*>(getRightLeg().getFiveBar()->getMotor1()),
        static_cast<aruwsrc::motor::Tmotor_AK809*>(getRightLeg().getFiveBar()->getMotor2())};
    int i = 0;
    for (i = 0; i < 4; i++)
    {  // 2. When the PID output exceeds some current, stop. Set the home, and Reset the PID to hold
        // the current position.
        float motorAngleSetpoint = legMotors[i]->getPositionUnwrapped() - modm::toRadian(5);

        float output = legHomingPid[i].runController(
            motorAngleSetpoint,
            legMotors[i]->getShaftRPM() * M_TWOPI / 60,
            dt);
        legMotors[i]->setDesiredOutput(output);
        if (compareFloatClose(legMotors[i]->getShaftRPM(), 0, 1e-3) &&
            !compareFloatClose(output, 0, STALL_CURRENT))
        {
            legStalled[i] = true;
            legMotors[i]->sendPositionHomeGetMessage();
        }
        else
        {
            legStalled[i] = false;
        }
    }

    if (legStalled[0] && legStalled[1] && legStalled[2] && legStalled[3])
    {
        legStalled[0] = false;
        legStalled[1] = false;
        legStalled[2] = false;
        legStalled[3] = false;
        homingState = HOMED;
    }
}

void BalancingChassisSubsystem::updateLegOdometry()
{
    leftLeg.setChassisPos(currentX, desiredX);
    rightLeg.setChassisPos(currentX, desiredX);
    leftLeg.setChassisSpeed(currentV);
    rightLeg.setChassisSpeed(currentV);
    leftLeg.setChassisYaw(desiredR, yawRate);
    // Right leg values are negated due to fun LQR logic
    rightLeg.setChassisYaw(-desiredR, -yawRate);
    leftLeg.setChassisAngle(pitch, pitchRate);
    rightLeg.setChassisAngle(pitch, pitchRate);
}
}  // namespace aruwsrc::chassis
