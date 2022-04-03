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
#include "constants/turret_controller_constants.hpp"

using namespace tap::motor;
using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
TurretSubsystem::TurretSubsystem(
    aruwsrc::Drivers *drivers,
    MotorInterface *pitchMotor,
    MotorInterface *yawMotor,
    bool limitYaw)
    : tap::control::turret::TurretSubsystemInterface(drivers),
      drivers(drivers),
      currPitchAngle(PITCH_START_ANGLE, 0.0f, 360.0f),
      currYawAngle(YAW_START_ANGLE, 0.0f, 360.0f),
      pitchEncoderWhenLastUpdated(PITCH_START_ENCODER_POSITION),
      yawEncoderWhenLastUpdated(YAW_START_ENCODER_POSITION),
      yawTarget(YAW_START_ANGLE, 0.0f, 360.0f),
      pitchTarget(PITCH_START_ANGLE, 0.0f, 360.0f),
      limitYaw(limitYaw),
      pitchMotor(pitchMotor),
      yawMotor(yawMotor)
{
}

void TurretSubsystem::initialize()
{
    yawMotor->initialize();
    pitchMotor->initialize();
}

float TurretSubsystem::getYawAngleFromCenter() const
{
    return ContiguousFloat(currYawAngle.getValue() - YAW_START_ANGLE, -180.0f, 180.0f).getValue();
}

float TurretSubsystem::getPitchAngleFromCenter() const
{
    return ContiguousFloat(currPitchAngle.getValue() - PITCH_START_ANGLE, -180.0f, 180.0f)
        .getValue();
}

void TurretSubsystem::refresh() { updateCurrentTurretAngles(); }

/**
 * Update the turret angle based on the current motor encoder value.
 */
static inline void updateCurrentMotorAngle(
    const MotorInterface *motor,
    const uint16_t startEncoderPosition,
    const float startAngle,
    ContiguousFloat *currAngle,
    uint16_t *encoderWhenLastUpdated)
{
    if (motor->isMotorOnline())
    {
        uint16_t encoder = motor->getEncoderWrapped();
        if (*encoderWhenLastUpdated == encoder)
        {
            return;
        }

        currAngle->setValue(
            DjiMotor::encoderToDegrees(static_cast<uint16_t>(encoder - startEncoderPosition)) +
            startAngle);

        *encoderWhenLastUpdated = encoder;
    }
    else
    {
        if (*encoderWhenLastUpdated == startEncoderPosition)
        {
            return;
        }

        currAngle->setValue(startAngle);
        *encoderWhenLastUpdated = startEncoderPosition;
    }
}

void TurretSubsystem::updateCurrentTurretAngles()
{
    updateCurrentMotorAngle(
        yawMotor,
        YAW_START_ENCODER_POSITION,
        YAW_START_ANGLE,
        &currYawAngle,
        &yawEncoderWhenLastUpdated);
    updateCurrentMotorAngle(
        pitchMotor,
        PITCH_START_ENCODER_POSITION,
        PITCH_START_ANGLE,
        &currPitchAngle,
        &pitchEncoderWhenLastUpdated);
}

/**
 * Sets the motor output. Limits the output if the turret is outside of the acceptable range.
 */
static inline void setMotorOutput(
    MotorInterface *motor,
    float out,
    const bool limitAngleInput,
    const ContiguousFloat &currMotorAngle,
    const float minAngle,
    const float maxAngle)
{
    out = limitVal(out, -TurretSubsystem::MAX_OUT_6020, TurretSubsystem::MAX_OUT_6020);

    if (motor->isMotorOnline())
    {
        // If angle equal to min or max angle, set desired output to desired output
        if (limitAngleInput)
        {
            // Wrap angle between min, max
            ContiguousFloat limitedVal(
                ContiguousFloat::limitValue(currMotorAngle, minAngle, maxAngle),
                0,
                360);

            if ((abs(limitedVal.difference(minAngle)) < 1E-5 && out < 0) ||
                (abs(limitedVal.difference(maxAngle)) < 1E-5 && out > 0))
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
        PITCH_MIN_ANGLE - 5.0f,
        PITCH_MAX_ANGLE + 5.0f);
}

void TurretSubsystem::setYawMotorOutput(float out)
{
    setMotorOutput(
        yawMotor,
        out,
        limitYaw,
        currYawAngle,
        YAW_MIN_ANGLE - 5.0f,
        YAW_MAX_ANGLE + 5.0f);
}

void TurretSubsystem::setYawSetpoint(float target)
{
    yawTarget.setValue(target);
    if (limitYaw)
    {
        yawTarget.setValue(ContiguousFloat::limitValue(yawTarget, YAW_MIN_ANGLE, YAW_MAX_ANGLE));
    }
}

void TurretSubsystem::setPitchSetpoint(float target)
{
    pitchTarget.setValue(target);
    pitchTarget.setValue(
        ContiguousFloat::limitValue(pitchTarget, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE));
}

void TurretSubsystem::onHardwareTestStart()
{
    pitchMotor->setDesiredOutput(0);
    yawMotor->setDesiredOutput(0);
}

}  // namespace aruwsrc::control::turret
