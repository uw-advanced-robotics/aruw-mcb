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
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

using namespace tap::motor;
using namespace tap::algorithms;
using namespace tap;

namespace aruwsrc::control::turret
{
TurretSubsystem::TurretSubsystem(
    Drivers *drivers,
    DjiMotor *pitchMotor,
    DjiMotor *yawMotor,
    bool limitYaw,
    bool chassisFrontBackIdentical)
    : tap::control::turret::TurretSubsystemInterface(drivers),
      currPitchAngle(PITCH_START_ANGLE, 0.0f, 360.0f),
      currYawAngle(YAW_START_ANGLE, 0.0f, 360.0f),
      pitchEncoderWhenUpdated(PITCH_START_ENCODER_POSITION),
      yawEncoderWhenUpdated(YAW_START_ENCODER_POSITION),
      yawTarget(YAW_START_ANGLE, 0.0f, 360.0f),
      pitchTarget(PITCH_START_ANGLE, 0.0f, 360.0f),
      limitYaw(limitYaw),
      chassisFrontBackIdentical(chassisFrontBackIdentical),
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
    const float wrapAngle = (!limitYaw && chassisFrontBackIdentical) ? 90.0f : 180.0f;
    return ContiguousFloat(currYawAngle.getValue() - YAW_START_ANGLE, -wrapAngle, wrapAngle)
        .getValue();
}

float TurretSubsystem::getPitchAngleFromCenter() const
{
    return ContiguousFloat(currPitchAngle.getValue() - PITCH_START_ANGLE, -180.0f, 180.0f)
        .getValue();
}

void TurretSubsystem::refresh() { updateCurrentTurretAngles(); }

void TurretSubsystem::updateCurrentTurretAngles()
{
    updateCurrentYawAngle();
    updateCurrentPitchAngle();
}

void TurretSubsystem::updateCurrentYawAngle()
{
    if (yawMotor->isMotorOnline())
    {
        uint16_t encoder = yawMotor->getEncoderWrapped();
        if (yawEncoderWhenUpdated == encoder)
        {
            return;
        }

        currYawAngle.setValue(
            DjiMotor::encoderToDegrees(
                static_cast<uint16_t>(encoder - YAW_START_ENCODER_POSITION)) +
            YAW_START_ANGLE);

        encoder = yawMotor->getEncoderWrapped();
    }
    else
    {
        if (yawEncoderWhenUpdated == YAW_START_ENCODER_POSITION)
        {
            return;
        }

        currYawAngle.setValue(YAW_START_ANGLE);
        yawEncoderWhenUpdated = YAW_START_ENCODER_POSITION;
    }
}

void TurretSubsystem::updateCurrentPitchAngle()
{
    if (pitchMotor->isMotorOnline())
    {
        uint16_t encoder = pitchMotor->getEncoderWrapped();

        if (pitchEncoderWhenUpdated == encoder)
        {
            return;
        }

        currPitchAngle.setValue(
            DjiMotor::encoderToDegrees(
                static_cast<uint16_t>(encoder - PITCH_START_ENCODER_POSITION)) +
            PITCH_START_ANGLE);

        pitchEncoderWhenUpdated = encoder;
    }
    else
    {
        if (pitchEncoderWhenUpdated == PITCH_START_ENCODER_POSITION)
        {
            return;
        }

        currPitchAngle.setValue(PITCH_START_ANGLE);
        pitchEncoderWhenUpdated = PITCH_START_ENCODER_POSITION;
    }
}

void TurretSubsystem::setPitchMotorOutput(float out)
{
    out = limitVal(out, -MAX_OUT_6020, MAX_OUT_6020);

    if (pitchMotor->isMotorOnline())
    {
        if ((currPitchAngle.getValue() > PITCH_MAX_ANGLE && out > 0) ||
            (currPitchAngle.getValue() < PITCH_MIN_ANGLE && out < 0))
        {
            pitchMotor->setDesiredOutput(0);
        }
        else
        {
            pitchMotor->setDesiredOutput(out);
        }
    }
}

void TurretSubsystem::setYawMotorOutput(float out)
{
    out = limitVal(out, -MAX_OUT_6020, MAX_OUT_6020);

    if (yawMotor->isMotorOnline())
    {
        if (limitYaw && ((currYawAngle.getValue() > YAW_MAX_ANGLE && out > 0) ||
                         (currYawAngle.getValue() < YAW_MIN_ANGLE && out < 0)))
        {
            yawMotor->setDesiredOutput(0);
        }
        else
        {
            yawMotor->setDesiredOutput(out);
        }
    }
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
