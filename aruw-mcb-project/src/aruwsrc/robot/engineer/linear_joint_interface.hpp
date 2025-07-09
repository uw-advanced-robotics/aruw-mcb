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

#ifndef LINEAR_JOINT_INTERFACE_HPP_
#define LINEAR_JOINT_INTERFACE_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

namespace aruwsrc::engineer
{
class LinearJointInterface
{
public:
    virtual void setSetpoint(float setpoint)
    {
        if (tap::algorithms::compareFloatClose(minSetpoint, maxSetpoint, epsilon))
            this->setpoint.setTarget(setpoint);
        else
            this->setpoint.setTarget(std::clamp(setpoint, minSetpoint, maxSetpoint));
    };

    inline void updateSetpoint() { setpoint.update(maxSetpointIncrement); }

    virtual float getSetpoint() { return setpoint.getValue(); }

    virtual float getPosition() = 0;

    virtual bool atSetpoint()
    {
        return tap::algorithms::compareFloatClose(setpoint.getTarget(), getPosition(), epsilon);
    };

    float getLowerBound() const { return minSetpoint; }

    float getUpperBound() const { return maxSetpoint; }

    void setLowerBound(float minSetpoint)
    {
        if (minSetpoint > this->maxSetpoint) return;
        this->minSetpoint = minSetpoint;
    }

    void setUpperBound(float maxSetpoint)
    {
        if (maxSetpoint < this->minSetpoint) return;
        this->maxSetpoint = maxSetpoint;
    }

protected:
    tap::algorithms::Ramp setpoint;
    float minSetpoint, maxSetpoint;
    const float epsilon, maxSetpointIncrement;

    LinearJointInterface(
        float minPosition = 0.0f,
        float maxSetpoint = 0.0f,
        float epsilon = 1e-4f,
        float initSetpoint = 0,
        float maxSetpointIncrement = FLT_MAX)
        : setpoint(initSetpoint),
          minSetpoint(minPosition),
          maxSetpoint(maxSetpoint),
          epsilon(epsilon),
          maxSetpointIncrement(maxSetpointIncrement){};
};

}  // namespace aruwsrc::engineer

#endif  // LINEAR_JOINT_INTERFACE_HPP_