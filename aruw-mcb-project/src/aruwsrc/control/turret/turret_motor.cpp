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
      lastUpdatedEncoderValue(config.startEncoderValue),
      chassisFrameUnwrappedMeasurement(config.startAngle)
{
    assert(motor != nullptr);
}

void TurretMotor::updateMotorAngle()
{
    if (isOnline())
    {
        uint16_t encoder = motor->getEncoderWrapped();
        if (lastUpdatedEncoderValue == encoder)
        {
            return;
        }

        lastUpdatedEncoderValue = encoder;

        chassisFrameMeasuredAngle.setValue(
            modm::toRadian(DjiMotor::encoderToDegrees(
                static_cast<uint16_t>(encoder - config.startEncoderValue))) +
            config.startAngle);

        chassisFrameUnwrappedMeasurement = modm::toRadian(
            DjiMotor::encoderToDegrees(
                motor->getEncoderUnwrapped() - static_cast<int64_t>(config.startEncoderValue)) +
            config.startAngle);
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
        // If angle equal to min or max angle, set desired output to desired output
        if (config.limitMotorAngles)
        {
            // Wrap angle between min, max
            ContiguousFloat limitedVal(
                ContiguousFloat::limitValue(
                    chassisFrameMeasuredAngle,
                    config.minAngle,
                    config.maxAngle),
                0,
                M_TWOPI);

            if ((abs(limitedVal.difference(config.minAngle)) < 1E-5 && out < 0) ||
                (abs(limitedVal.difference(config.maxAngle)) < 1E-5 && out > 0))
            {
                motor->setDesiredOutput(0);
                return;
            }
        }

        motor->setDesiredOutput(out);
    }
}

void TurretMotor::setChassisFrameSetpoint(float setpoint)
{
    chassisFrameSetpoint.setValue(setpoint);
    if (config.limitMotorAngles)
    {
        chassisFrameSetpoint.setValue(
            ContiguousFloat::limitValue(chassisFrameSetpoint, config.minAngle, config.maxAngle));
    }
}

float TurretMotor::getValidChassisMeasurementError() const
{
    return getValidMinError(chassisFrameUnwrappedMeasurement);
}

float TurretMotor::getValidMinError(const float measurement) const
{
    if (config.limitMotorAngles)
    {
        return chassisFrameSetpoint.getValue() - measurement;
    }
    else
    {
        // equivalent to this - other
        return -chassisFrameSetpoint.difference(measurement);
    }
}
}  // namespace aruwsrc::control::turret
