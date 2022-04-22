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
      lastUpdatedEncoderValue(config.startEncoderValue),
      chassisFrameUnwrappedMeasurement(config.startAngle)
{
    assert(motor != nullptr);
}

void TurretMotor::updateMotorAngle()
{
    if (isOnline())
    {
        int64_t encoderUnwrapped = motor->getEncoderUnwrapped();
        if (lastUpdatedEncoderValue == encoderUnwrapped)
        {
            return;
        }

        lastUpdatedEncoderValue = encoderUnwrapped;

        chassisFrameUnwrappedMeasurement =
            static_cast<float>(encoderUnwrapped - static_cast<int64_t>(config.startEncoderValue)) *
                M_TWOPI / static_cast<float>(DjiMotor::ENC_RESOLUTION) +
            config.startAngle;

        chassisFrameMeasuredAngle.setValue(chassisFrameUnwrappedMeasurement);
    }
    else
    {
        if (lastUpdatedEncoderValue == config.startEncoderValue)
        {
            return;
        }

        lastUpdatedEncoderValue = config.startEncoderValue;

        chassisFrameMeasuredAngle.setValue(config.startAngle);
    }
}

void TurretMotor::setMotorOutput(float out)
{
    out = limitVal(out, -MAX_OUT_6020, MAX_OUT_6020);

    if (motor->isMotorOnline())
    {
        motor->setDesiredOutput(out);
    }
}

void TurretMotor::setChassisFrameSetpoint(float setpoint)
{
    if (config.limitMotorAngles)
    {
        setpoint = limitVal(setpoint, config.minAngle, config.maxAngle);
    }

    chassisFrameSetpoint = setpoint;
}

float TurretMotor::getValidChassisMeasurementError() const
{
    return getValidMinError(chassisFrameUnwrappedMeasurement);
}

float TurretMotor::getValidMinError(const float measurement) const
{
    if (config.limitMotorAngles)
    {
        // the error is absolute
        return chassisFrameSetpoint - measurement;
    }
    else
    {
        // the error can be wrapped around the unit circle
        // equivalent to this - other
        return ContiguousFloat(measurement, 0, M_TWOPI).difference(chassisFrameSetpoint);
    }
}
}  // namespace aruwsrc::control::turret
