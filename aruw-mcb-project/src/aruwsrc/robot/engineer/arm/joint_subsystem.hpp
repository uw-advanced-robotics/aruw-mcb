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
#ifndef JOINT_SUBSYSTEM_HPP_
#define JOINT_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/math/filter/pid.hpp"

namespace aruwsrc::engineer::arm
{
struct JointSubsystemConfig
{
    float p;
    float i;
    float d;
    float maxErrorSum;
    float maxOutput;  // This value does not include the feedforward term
    uint32_t feedforward = 0;

    float setpointTolerance = 0.0f;
    float setpointToEncoderScalar = 1.0f;

    float lowerBound;
    float upperBound;
};

/**
 * Abstract class for creating a linear-axis subsystem.
 * Contains a positional PID controller meant to control a motor.
 */
class JointSubsystem : public tap::control::Subsystem
{
public:
    JointSubsystem(
        tap::Drivers* drivers,
        const JointSubsystemConfig& config,
        tap::motor::MotorInterface* motor,
        const char* jointName)
        : tap::control::Subsystem(drivers),
          motor(motor),
          config(config),
          positionPid(config.p, config.i, config.d, config.maxErrorSum, config.maxOutput),
          name(jointName)
    {
    }

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override { motor->setDesiredOutput(0); }

    void setSetpoint(float setpoint);

    float getSetpoint();

    float getPosition();

    inline bool atSetpoint()
    {
        return std::abs(positionPid.getLastError()) <= config.setpointTolerance;
    }

    bool isOnline() { return motor->isMotorOnline(); }

    const char* getName() const override { return name; }

protected:
    tap::motor::MotorInterface* motor;
    JointSubsystemConfig config;

    modm::Pid<float> positionPid;
    float setpoint;

    const char* name;

};  // class JointSubsystem

}  // namespace aruwsrc::engineer::arm
#endif  // JOINT_SUBSYSTEM_HPP_
