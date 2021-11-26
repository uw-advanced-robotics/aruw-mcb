/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "turret_subsystem.hpp"

#include <algorithm>
#include <cfloat>
#include <random>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::motor;

namespace aruwsrc::control::turret
{
TurretSubsystem::TurretSubsystem(aruwsrc::Drivers* drivers, bool limitYaw)
    : tap::control::turret::TurretSubsystemInterface(drivers),
      currPitchAngle(0.0f, 0.0f, 360.0f),
      currYawAngle(0.0f, 0.0f, 360.0f),
      yawTarget(TURRET_START_ANGLE, 0.0f, 360.0f),
      pitchTarget(TURRET_START_ANGLE, 0.0f, 360.0f),
      limitYaw(limitYaw),
      pitchMotor(drivers, PITCH_MOTOR_ID, CAN_BUS_MOTORS, true, "pitch motor"),
      yawMotor(drivers, YAW_MOTOR_ID, CAN_BUS_MOTORS, false, "yaw motor")
{
}

void TurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
}

float TurretSubsystem::getYawAngleFromCenter() const
{
    tap::algorithms::ContiguousFloat yawAngleFromCenter(
        currYawAngle.getValue() - TURRET_START_ANGLE,
        -180.0f,
        180.0f);
    return yawAngleFromCenter.getValue();
}

float TurretSubsystem::getPitchAngleFromCenter() const
{
    tap::algorithms::ContiguousFloat yawAngleFromCenter(
        currPitchAngle.getValue() - TURRET_START_ANGLE,
        -180.0f,
        180.0f);
    return yawAngleFromCenter.getValue();
}

void TurretSubsystem::refresh() { updateCurrentTurretAngles(); }

void TurretSubsystem::updateCurrentTurretAngles()
{
    updateCurrentYawAngle();
    updateCurrentPitchAngle();
}

void TurretSubsystem::updateCurrentYawAngle()
{
    if (yawMotor.isMotorOnline())
    {
        currYawAngle.setValue(
            DjiMotor::encoderToDegrees(
                static_cast<uint16_t>(yawMotor.getEncoderWrapped() - YAW_START_ENCODER_POSITION)) +
            TURRET_START_ANGLE);
    }
    else
    {
        currYawAngle.setValue(TURRET_START_ANGLE);
    }
}

void TurretSubsystem::updateCurrentPitchAngle()
{
    if (pitchMotor.isMotorOnline())
    {
        currPitchAngle.setValue(
            DjiMotor::encoderToDegrees(static_cast<uint16_t>(
                pitchMotor.getEncoderWrapped() - PITCH_START_ENCODER_POSITION)) +
            TURRET_START_ANGLE);
    }
    else
    {
        currPitchAngle.setValue(TURRET_START_ANGLE);
    }
}

void TurretSubsystem::setPitchMotorOutput(float out)
{
    if (out > INT32_MAX || out < INT32_MIN)
    {
        RAISE_ERROR(drivers, "pitch motor output invalid");
        return;
    }
    if (pitchMotor.isMotorOnline())
    {
        if ((getPitchAngleFromCenter() + TURRET_START_ANGLE > TURRET_PITCH_MAX_ANGLE && out > 0) ||
            (getPitchAngleFromCenter() + TURRET_START_ANGLE < TURRET_PITCH_MIN_ANGLE && out < 0))
        {
            pitchMotor.setDesiredOutput(0);
        }
        else
        {
            pitchMotor.setDesiredOutput(out);
        }
    }
}

void TurretSubsystem::setYawMotorOutput(float out)
{
    if (out > INT32_MAX || out < INT32_MIN)
    {
        RAISE_ERROR(drivers, "yaw motor output invalid");
        return;
    }
    if (yawMotor.isMotorOnline())
    {
        if (limitYaw &&
            ((getYawAngleFromCenter() + TURRET_START_ANGLE > TURRET_YAW_MAX_ANGLE && out > 0) ||
             (getYawAngleFromCenter() + TURRET_START_ANGLE < TURRET_YAW_MIN_ANGLE && out < 0)))
        {
            yawMotor.setDesiredOutput(0);
        }
        else
        {
            yawMotor.setDesiredOutput(out);
        }
    }
}

void TurretSubsystem::setYawSetpoint(float target)
{
    yawTarget.setValue(target);
    if (limitYaw)
    {
        yawTarget.setValue(tap::algorithms::ContiguousFloat::limitValue(
            yawTarget,
            TURRET_YAW_MIN_ANGLE,
            TURRET_YAW_MAX_ANGLE));
    }
}

void TurretSubsystem::setPitchSetpoint(float target)
{
    pitchTarget.setValue(target);
    pitchTarget.setValue(tap::algorithms::ContiguousFloat::limitValue(
        pitchTarget,
        TURRET_PITCH_MIN_ANGLE,
        TURRET_PITCH_MAX_ANGLE));
}

float TurretSubsystem::yawFeedForwardCalculation(float desiredChassisRotation)
{
    float chassisRotationFeedForward = FEED_FORWARD_KP * desiredChassisRotation;

    if ((chassisRotationFeedForward > 0.0f &&
         getCurrentYawValue().getValue() > TurretSubsystem::TURRET_YAW_MAX_ANGLE) ||
        (chassisRotationFeedForward < 0.0f &&
         getCurrentYawValue().getValue() < TurretSubsystem::TURRET_YAW_MIN_ANGLE))
    {
        chassisRotationFeedForward = 0.0f;
    }
    return tap::algorithms::limitVal<float>(
        chassisRotationFeedForward,
        -FEED_FORWARD_MAX_OUTPUT,
        FEED_FORWARD_MAX_OUTPUT);
}

void TurretSubsystem::onHardwareTestStart()
{
    pitchMotor.setDesiredOutput(0);
    yawMotor.setDesiredOutput(0);
}

}  // namespace aruwsrc::control::turret
