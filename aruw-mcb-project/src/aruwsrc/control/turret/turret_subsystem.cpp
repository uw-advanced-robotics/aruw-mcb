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
using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
TurretSubsystem::TurretSubsystem(
    aruwsrc::Drivers *drivers,
    MotorInterface *pitchMotor,
    MotorInterface *yawMotor,
    const TurretSubsystemConfig &turretConfig)
    : tap::control::turret::TurretSubsystemInterface(drivers),
      drivers(drivers),
      turretConfig(turretConfig),
      yawSetpoint(turretConfig.yawStartAngle),
      yawSetpointWrapped(turretConfig.yawStartAngle, 0, 360),
      currYawAngle(turretConfig.yawStartAngle),
      currYawAngleWrapped(turretConfig.yawStartAngle, 0, 360),
      pitchSetpoint(turretConfig.pitchStartAngle),
      currPitchAngle(turretConfig.pitchStartAngle),
      pitchEncoderWhenLastUpdated(turretConfig.pitchStartEncoderValue),
      yawEncoderWhenLastUpdated(turretConfig.yawStartEncoderValue),
      pitchMotor(pitchMotor),
      yawMotor(yawMotor)
{
    assert(drivers != nullptr);
    assert(pitchMotor != nullptr);
    assert(yawMotor != nullptr);
}

void TurretSubsystem::initialize()
{
    yawMotor->initialize();
    pitchMotor->initialize();
}

float TurretSubsystem::getYawAngleFromCenter() const
{
    float yawAngle = turretConfig.limitYaw ? currYawAngle : currYawAngleWrapped.getValue();
    return ContiguousFloat(yawAngle - turretConfig.yawStartAngle, -180.0f, 180.0f).getValue();
}

float TurretSubsystem::getPitchAngleFromCenter() const
{
    return ContiguousFloat(currPitchAngle - turretConfig.pitchStartAngle, -180.0f, 180.0f)
        .getValue();
}

void TurretSubsystem::refresh() { updateCurrentTurretAngles(); }

/**
 * Update the turret angle based on the current motor encoder value.
 */
static inline float updateCurrentMotorAngle(
    const float currMotorAngle,
    const MotorInterface &motor,
    const uint16_t startEncoderPosition,
    const float startAngle,
    uint16_t *encoderWhenLastUpdated)
{
    if (motor.isMotorOnline())
    {
        uint16_t encoder = motor.getEncoderUnwrapped();
        if (*encoderWhenLastUpdated == encoder)
        {
            return currMotorAngle;
        }

        *encoderWhenLastUpdated = encoder;

        return DjiMotor::encoderToDegrees(static_cast<uint16_t>(encoder - startEncoderPosition)) +
               startAngle;
    }
    else
    {
        if (*encoderWhenLastUpdated == startEncoderPosition)
        {
            return currMotorAngle;
        }

        *encoderWhenLastUpdated = startEncoderPosition;

        return startAngle;
    }
}

void TurretSubsystem::updateCurrentTurretAngles()
{
    float yawAngle = updateCurrentMotorAngle(
        turretConfig.limitYaw ? yawSetpoint : yawSetpointWrapped.getValue(),
        *yawMotor,
        turretConfig.yawStartEncoderValue,
        turretConfig.yawStartAngle,
        &yawEncoderWhenLastUpdated);

    if (turretConfig.limitYaw)
    {
        yawSetpoint = yawAngle;
    }
    else
    {
        yawSetpointWrapped.setValue(yawAngle);
    }

    pitchSetpoint = updateCurrentMotorAngle(
        pitchSetpoint,
        *pitchMotor,
        turretConfig.pitchStartEncoderValue,
        turretConfig.pitchStartAngle,
        &pitchEncoderWhenLastUpdated);
}

/**
 * Sets the motor output. Limits the output if the turret is outside of the acceptable range.
 */
static inline void setMotorOutput(
    MotorInterface *motor,
    float out,
    const bool limitAngleInput,
    const float currMotorAngle,
    const float minAngle,
    const float maxAngle)
{
    out = limitVal(out, -TurretSubsystem::MAX_OUT_6020, TurretSubsystem::MAX_OUT_6020);

    if (motor->isMotorOnline())
    {
        // If angle equal to min or max angle, set desired output to desired output
        if (limitAngleInput)
        {
            if ((minAngle - currMotorAngle < 0 && out < 0) ||
                (maxAngle - currMotorAngle > 0 && out > 0))
            {
                motor->setDesiredOutput(0);
                return;
            }
        }

        motor->setDesiredOutput(out);
    }
}

void TurretSubsystem::setPitchMotorOutput(float out)
{
    setMotorOutput(
        pitchMotor,
        out,
        true,
        currPitchAngle,
        turretConfig.pitchMinAngle - 5.0f,
        turretConfig.pitchMaxAngle + 5.0f);
}

void TurretSubsystem::setYawMotorOutput(float out)
{
    setMotorOutput(
        yawMotor,
        out,
        turretConfig.limitYaw,
        turretConfig.limitYaw ? currYawAngleWrapped.getValue() : currYawAngle,
        turretConfig.yawMinAngle - 5.0f,
        turretConfig.yawMaxAngle + 5.0f);
}

void TurretSubsystem::setYawSetpoint(float target)
{
    if (turretConfig.limitYaw)
    {
        yawSetpoint = limitVal(target, turretConfig.yawMinAngle, turretConfig.yawMaxAngle);
    }
    else
    {
        yawSetpointWrapped.setValue(target);
    }
}

void TurretSubsystem::setPitchSetpoint(float target)
{
    pitchSetpoint = limitVal(target, turretConfig.pitchMinAngle, turretConfig.pitchMaxAngle);
}

void TurretSubsystem::onHardwareTestStart()
{
    pitchMotor->setDesiredOutput(0);
    yawMotor->setDesiredOutput(0);
}

}  // namespace aruwsrc::control::turret
