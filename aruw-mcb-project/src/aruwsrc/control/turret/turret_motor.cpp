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
      chassisFrameSetpoint(config.startAngle, 0, M_TWOPI),
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

        if (startEncoderOffset == INT16_MIN)
        {
            int encoderDiff =
                static_cast<int>(config.startEncoderValue) - static_cast<int>(encoderUnwrapped);

            if (encoderDiff < -static_cast<int>(DjiMotor::ENC_RESOLUTION / 2))
            {
                // encoder offset by 1 rev in negative direction
                startEncoderOffset = -DjiMotor::ENC_RESOLUTION;
            }
            else if (encoderDiff > DjiMotor::ENC_RESOLUTION / 2)
            {
                // offset by 1 rev in positive direction
                startEncoderOffset = DjiMotor::ENC_RESOLUTION;
            }
            else
            {
                // no offset necessary
                startEncoderOffset = 0;
            }
        }

        if (lastUpdatedEncoderValue == encoderUnwrapped)
        {
            return;
        }

        lastUpdatedEncoderValue = encoderUnwrapped;

        chassisFrameUnwrappedMeasurement =
            static_cast<float>(
                encoderUnwrapped - static_cast<int64_t>(config.startEncoderValue) +
                startEncoderOffset) *
                M_TWOPI / static_cast<float>(DjiMotor::ENC_RESOLUTION) +
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
        startEncoderOffset = INT16_MIN;

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

void TurretMotor::setChassisFrameSetpoint(WrappedFloat setpoint)
{
    chassisFrameSetpoint = setpoint;

    if (config.limitMotorAngles)
    {
        int status;
        chassisFrameSetpoint = Angle(WrappedFloat::limitValue(
            chassisFrameSetpoint,
            config.minAngle,
            config.maxAngle,
            &status));
    }
}

float TurretMotor::getValidChassisMeasurementError() const
{
    return getValidMinError(chassisFrameSetpoint, chassisFrameMeasuredAngle);
}

// unused?
float TurretMotor::getValidChassisMeasurementErrorWrapped() const
{
    // equivalent to this - other
    return chassisFrameMeasuredAngle.minDifference(chassisFrameSetpoint);
}

float TurretMotor::getValidMinError(const WrappedFloat setpoint, const WrappedFloat measurement)
    const
{
    if (config.limitMotorAngles)
    {
        int limitStatus;
        WrappedFloat::limitValue(measurement, config.minAngle, config.maxAngle, &limitStatus);
        if (limitStatus != 0)
            return measurement.minDifference(setpoint);  // measurement is somehow out of bounds

        return (setpoint - config.minAngle).getWrappedValue() -
               (measurement - config.minAngle).getWrappedValue();
    }
    else
    {
        // the error can be wrapped around the unit circle
        // equivalent to this - other
        return measurement.minDifference(setpoint);
    }
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
