/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "turret_motor.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/motor/dji_motor.hpp"

using namespace tap::motor;
using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
TurretMotor::TurretMotor(tap::motor::MotorInterface *motor, const TurretMotorConfig &motorConfig)
    : config(motorConfig),
      motor(motor),
      chassisFrameSetpoint(config.startAngle),
      chassisFrameMeasuredAngle(config.startAngle, 0, M_TWOPI),
      chassisFrameUnwrappedMeasurement(config.startAngle),
      lastUpdatedEncoderValue(config.startEncoderValue)
{
    assert(config.minAngle <= config.maxAngle);
    assert(motor != nullptr);
}

void TurretMotor::updateMotorAngle()
{
    if (isOnline())
    {
        int64_t encoderUnwrapped = motor->getEncoderUnwrapped();

        float minAngleDiffFromStart = modm::toDegree(config.startAngle - config.minAngle);
        minAngleDiffFromStart =
            minAngleDiffFromStart < 0 ? -minAngleDiffFromStart : minAngleDiffFromStart;  // abs val

        float maxAngleDiffFromStart = modm::toDegree(config.maxAngle - config.startAngle);
        maxAngleDiffFromStart =
            maxAngleDiffFromStart < 0 ? -maxAngleDiffFromStart : maxAngleDiffFromStart;  // abs val

        minAngleEncoder =
            config.startEncoderValue - DjiMotor::degreesToEncoder<int64_t>(minAngleDiffFromStart);
        maxAngleEncoder =
            config.startEncoderValue + DjiMotor::degreesToEncoder<int64_t>(maxAngleDiffFromStart);

        if (startValueNeedsCorrection)
        {
            if (config.limitMotorAngles)
            {
                if (encoderUnwrapped >= minAngleEncoder && encoderUnwrapped <= maxAngleEncoder)
                {
                    adjustedStartEncoderValue = config.startEncoderValue;
                }
                else if (encoderUnwrapped > maxAngleEncoder)
                {
                    adjustedStartEncoderValue =
                        config.startEncoderValue +
                        DjiMotor::ENC_RESOLUTION;  // currently assume within 1 revolution from
                                                   // expected
                }
                else if (encoderUnwrapped < minAngleEncoder)
                {
                    adjustedStartEncoderValue = config.startEncoderValue - DjiMotor::ENC_RESOLUTION;
                }
                else
                {
                    adjustedStartEncoderValue = config.startEncoderValue;
                }
            }
            else
            {
                float positionRad = modm::toRadian(DjiMotor::encoderToDegrees(encoderUnwrapped));
                float adjustedStartAngleDegrees =
                    modm::toDegree(getClosestNonNormalizedSetpointToMeasurement(
                        positionRad,
                        startEncoderValueRad));
                adjustedStartEncoderValue =
                    DjiMotor::degreesToEncoder<int64_t>(adjustedStartAngleDegrees);
            }
            startValueNeedsCorrection = false;
        }

        if (lastUpdatedEncoderValue == encoderUnwrapped)
        {
            return;
        }

        lastUpdatedEncoderValue = encoderUnwrapped;

        chassisFrameUnwrappedMeasurement =
            static_cast<float>(encoderUnwrapped - adjustedStartEncoderValue) * M_TWOPI /
                static_cast<float>(DjiMotor::ENC_RESOLUTION) +
            config.startAngle;

        chassisFrameMeasuredAngle.setWrappedValue(chassisFrameUnwrappedMeasurement);
    }
    else
    {
        if (lastUpdatedEncoderValue == config.startEncoderValue)
        {
            return;
        }

        lastUpdatedEncoderValue = config.startEncoderValue;
        startValueNeedsCorrection = true;

        chassisFrameMeasuredAngle.setWrappedValue(config.startAngle);
    }
}

void TurretMotor::setMotorOutput(float out)
{
    out = limitVal(out, -MAX_OUT_6020, MAX_OUT_6020);

    if (motor->isMotorOnline())
    {
        motor->setDesiredOutput(out);
    }
    else
    {
        motor->setDesiredOutput(0);
    }
}

void TurretMotor::setChassisFrameSetpoint(float setpoint)
{
    chassisFrameSetpoint = setpoint;

    if (config.limitMotorAngles)
    {
        chassisFrameSetpoint = limitVal(chassisFrameSetpoint, config.minAngle, config.maxAngle);
    }
}

float TurretMotor::getValidChassisMeasurementError() const
{
    return getValidMinError(chassisFrameSetpoint, chassisFrameUnwrappedMeasurement);
}

float TurretMotor::getValidChassisMeasurementErrorWrapped() const
{
    // equivalent to this - other
    return WrappedFloat(chassisFrameUnwrappedMeasurement, 0, M_TWOPI)
        .minDifference(chassisFrameSetpoint);
}

float TurretMotor::getValidMinError(const float setpoint, const float measurement) const
{
    if (config.limitMotorAngles)
    {
        // the error is absolute
        return setpoint - measurement;
    }
    else
    {
        // the error can be wrapped around the unit circle
        // equivalent to this - other
        return WrappedFloat(measurement, 0, M_TWOPI).minDifference(setpoint);
    }
}

float TurretMotor::getClosestNonNormalizedSetpointToMeasurement(float measurement, float setpoint)
{
    return WrappedFloat(WrappedFloat(measurement, 0, M_TWOPI).minDifference(setpoint), -M_PI, M_PI)
               .getWrappedValue() +
           measurement;
}

float TurretMotor::getSetpointWithinTurretRange(float setpoint) const
{
    if (setpoint < config.minAngle)
    {
        float newSetpoint = setpoint;
        while (newSetpoint < config.minAngle)
        {
            newSetpoint += M_TWOPI;
        }
        return newSetpoint <= config.maxAngle ? newSetpoint : setpoint;
    }

    if (setpoint > config.maxAngle)
    {
        float newSetpoint = setpoint;
        while (newSetpoint > config.maxAngle)
        {
            newSetpoint -= M_TWOPI;
        }
        return newSetpoint >= config.minAngle ? newSetpoint : setpoint;
    }

    return setpoint;
}

}  // namespace aruwsrc::control::turret
