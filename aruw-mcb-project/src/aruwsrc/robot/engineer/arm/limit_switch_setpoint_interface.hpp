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

#ifndef LIMIT_SWITCH_SETPOINT_INTERFACE_HPP_
#define LIMIT_SWITCH_SETPOINT_INTERFACE_HPP_

#include "aruwsrc/control/bounded-subsystem/one_sided_bounded_subsystem_interface.hpp"
#include "aruwsrc/control/bounded-subsystem/trigger/trigger_interface.hpp"
#include "aruwsrc/robot/engineer/arm/linear_joint_interface.hpp"

namespace aruwsrc::engineer
{
enum class PIDState
{
    POSITION_PID,
    VELOCITY_PID,
    NONE
};

class LimitSwitchSetpointInterface : public aruwsrc::control::OneSidedBoundedSubsystemInterface,
                                     public LinearJointInterface
{
public:
    void setPIDState(PIDState state) { pidState = state; }

    PIDState getPIDState() { return pidState; }

    bool isTriggered() { return trigger.isTriggered(); }

    virtual void setDesiredOutput(int16_t output) = 0;

    virtual uint64_t getLowerBound() const override
    {
        return getMinSetpoint() / M_TWOPI * 4096;  // todo
    }

    virtual uint64_t getUpperBound() const override
    {
        return getMaxSetpoint() / M_TWOPI * 4096;  // todo
    }

    void setUpperBound(uint64_t encoderPosition) { maxSetpoint = encoderPosition * M_TWOPI / 4096; }

    void setLowerBound(uint64_t encoderPosition) { minSetpoint = encoderPosition * M_TWOPI / 4096; }

    bool homedAndBounded() const
    {
        return calibrationState == CalibrationState::CALIBRATION_COMPLETE;
    }

protected:
    LimitSwitchSetpointInterface(
        tap::Drivers *drivers,
        aruwsrc::control::TriggerInterface &trigger,
        float lowerBound = 0.0f,
        float upperBound = 0.0f,
        float epsilon = 0.5f)
        : OneSidedBoundedSubsystemInterface(drivers, trigger, 0),
          LinearJointInterface(lowerBound, upperBound, epsilon)
    {
    }

    PIDState pidState = PIDState::NONE;
    CalibrationState caliState = CalibrationState::AWAITING_CALIBRATE;
};
}  // namespace aruwsrc::engineer

#endif