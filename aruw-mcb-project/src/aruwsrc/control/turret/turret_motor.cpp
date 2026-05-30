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
TurretMotor::TurretMotor(
    tap::motor::MotorInterface* motor,
    const TurretMotorConfig& motorConfig,
    float (*minLimitFunc)(),
    float (*maxLimitFunc)())
    : config(motorConfig),
      motor(motor),
      chassisFrameSetpoint(Angle(config.startAngle)),
      chassisFrameMeasuredAngle(Angle(config.startAngle)),
      minLimitFunc(minLimitFunc),
      maxLimitFunc(maxLimitFunc)
{
    assert(config.minAngle <= config.maxAngle);
    assert(motor != nullptr);
}

void TurretMotor::updateMotorAngle()
{
    if (isOnline())
    {
        float chassisFrameUnwrappedMeasurement =
            motor->getEncoder()->getPosition().getUnwrappedValue() + config.startAngle;

        chassisFrameMeasuredAngle.setUnwrappedValue(chassisFrameUnwrappedMeasurement);
    }
    else
    {
        chassisFrameMeasuredAngle.setUnwrappedValue(config.startAngle);
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
        float minAngle = getMinLimit();
        float maxAngle = getMaxLimit();

        int status;
        chassisFrameSetpoint =
            Angle(WrappedFloat::limitValue(chassisFrameSetpoint, minAngle, maxAngle, &status));
    }

}

float TurretMotor::getValidChassisMeasurementError() const
{
    return getValidMinError(chassisFrameSetpoint, chassisFrameMeasuredAngle);
}

float TurretMotor::getValidMinError(const WrappedFloat setpoint, const WrappedFloat measurement)
    const
{
    if (config.limitMotorAngles)
    {
        float minAngle = getMinLimit();
        float maxAngle = getMaxLimit();

        float pos =
            WrappedFloat::rangeOverlap(measurement, setpoint, Angle(maxAngle), Angle(minAngle));
        float neg =
            WrappedFloat::rangeOverlap(setpoint, measurement, Angle(maxAngle), Angle(minAngle));

        if (pos < neg)
        {
            return (setpoint - measurement).getWrappedValue();
        }
        else if (pos > neg)
        {
            return (setpoint - measurement).getWrappedValue() - M_TWOPI;
        }
    }

    // the error can be wrapped around the unit circle
    // equivalent to this - other
    return measurement.minDifference(setpoint);
}

}  // namespace aruwsrc::control::turret
