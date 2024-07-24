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
#ifndef ARM_SUBSYSTEM_HPP_
#define ARM_SUBSYSTEM_HPP_

#include "linear_subsystem.hpp"

namespace aruwsrc::robot::engineer
{

class XAxisSubsystem : public LinearSubsystem
{
public:
    XAxisSubsystem(
        tap::Drivers* drivers,
        const LinearSubsystemConfig& config,
        tap::motor::MotorInterface* motor)
        : LinearSubsystem(drivers, config, motor)
    {
    }

    void setSetpoint(float setpoint) override { setpoint = setpoint / SETPOINT_M_TO_ENCODER_TICKS; }

    float getPosition() override
    {
        return motor->getEncoderUnwrapped() * SETPOINT_M_TO_ENCODER_TICKS;
    }

    const char* getName() const override { return "X Axis Subsystem"; }

private:
    static constexpr float SETPOINT_M_TO_ENCODER_TICKS = 1.0f;

};  // class XAxisSubsystem

class LiftSubsystem : public LinearSubsystem
{
public:
    LiftSubsystem(
        tap::Drivers* drivers,
        const LinearSubsystemConfig& config,
        tap::motor::MotorInterface* motor)
        : LinearSubsystem(drivers, config, motor)
    {
    }

    void setSetpoint(float setpoint) override { setpoint = setpoint / SETPOINT_M_TO_ENCODER_TICKS; }

    float getPosition() override
    {
        return motor->getEncoderUnwrapped() * SETPOINT_M_TO_ENCODER_TICKS;
    }

    const char* getName() const override { return "Lift Subsystem"; }

private:
    static constexpr float SETPOINT_M_TO_ENCODER_TICKS = 1.0f;

};  // class LiftSubsystem

class YawSubsystem : public LinearSubsystem
{
public:
    YawSubsystem(
        tap::Drivers* drivers,
        const LinearSubsystemConfig& config,
        tap::motor::MotorInterface* motor)
        : LinearSubsystem(drivers, config, motor)
    {
    }

    void setSetpoint(float setpoint) override { setpoint = setpoint / SETPOINT_DEG_TO_ENCODER_TICKS; }

    float getPosition() override
    {
        return motor->getEncoderUnwrapped() * SETPOINT_DEG_TO_ENCODER_TICKS;
    }

    const char* getName() const override { return "Yaw Subsystem"; }

private:
    static constexpr float SETPOINT_DEG_TO_ENCODER_TICKS = 1.0f;

};  // class YawSubsystem

}  // namespace aruwsrc::robot::engineer
#endif  // ARM_SUBSYSTEM_HPP_
