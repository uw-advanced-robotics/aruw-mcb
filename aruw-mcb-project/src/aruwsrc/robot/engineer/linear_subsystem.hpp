/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef LINEAR_SUBSYSTEM_HPP_
#define LINEAR_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/math/filter/pid.hpp"

namespace aruwsrc::robot::engineer
{
struct LinearSubsystemConfig
{
    float p;
    float i;
    float d;
    float maxErrorSum;
    float maxOutput;  // This value does not include the feedforward term
    uint32_t feedforward;
    float setpointTolerance;
};

/**
 * Abstract class for creating a linear-axis subsystem
 */
class LinearSubsystem : public tap::control::Subsystem
{
public:
    LinearSubsystem(tap::Drivers* drivers, const LinearSubsystemConfig& config, tap::motor::MotorInterface* motor);

    // Primary method to be overriden by subclasses so that setpoint units can be converted
    virtual void setSetpoint(float setpoint);

    void refresh() override;

    void refreshSafeDisconnect() override;

    inline bool atSetpoint()
    {
        return std::abs(positionPid.getLastError()) <= setpointTolerance;
    }

private:
    LinearSubsystemConfig config;
    tap::motor::MotorInterface* motor;

    modm::Pid<float> positionPid;
    float setpoint;
    float setpointTolerance;
    uint32_t feedforward;

};  // class LinearSubsystem

}  // namespace aruwsrc::robot::engineer
#endif  // LINEAR_SUBSYSTEM_HPP_
