/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef LINEAR_SETPOINT_INTERFACE_HPP_
#define LINEAR_SETPOINT_INTERFACE_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

namespace aruwsrc::control
{
class LinearSetpointInterface
{
public:
    struct Config
    {
        float lowerBound = 0.0f, upperBound = 0.0f;
        float epsilon = 1e-4;
        float maxSetpointIncrement = FLT_MAX;
        float initSetpoint = 0;
    };

    LinearSetpointInterface(Config config)
        : setpoint(config.initSetpoint),
          lowerBound(config.lowerBound),
          upperBound(config.upperBound),
          epsilon(config.epsilon),
          maxSetpointIncrement(config.maxSetpointIncrement){};

    virtual void setSetpoint(float setpoint)
    {
        if (tap::algorithms::compareFloatClose(lowerBound, upperBound, epsilon))
            this->setpoint.setTarget(setpoint);
        else
            this->setpoint.setTarget(std::clamp(setpoint, lowerBound, upperBound));
    };

    inline void updateSetpoint() { setpoint.update(maxSetpointIncrement); }

    float getSetpoint() const { return setpoint.getValue(); }

    virtual float getPosition() const = 0;

    virtual float getVelocity() const = 0;

    virtual bool atSetpoint()
    {
        return tap::algorithms::compareFloatClose(setpoint.getTarget(), getPosition(), epsilon);
    };

    float getLowerBound() const { return lowerBound; }

    float getUpperBound() const { return upperBound; }

    void setLowerBound(float lowerBound)
    {
        if (lowerBound > this->upperBound) return;
        this->lowerBound = lowerBound;
    }

    void setUpperBound(float upperBound)
    {
        if (upperBound < this->lowerBound) return;
        this->upperBound = upperBound;
    }

protected:
    tap::algorithms::Ramp setpoint;
    float lowerBound, upperBound;
    const float epsilon, maxSetpointIncrement;
};

}  // namespace aruwsrc::control

#endif  // LINEAR_SETPOINT_INTERFACE_HPP_