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

#ifndef WRIST_SUBSYSTEM_HPP_
#define WRIST_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"
#include "tap/util_macros.hpp"

namespace aruwsrc
{
class Drivers;

namespace engineer
{
class WristSubsystem : public tap::control::Subsystem
{
private:
    tap::motor::MotorInterface &motorLeft, &motorRight;
    tap::algorithms::SmoothPid pidPitch, pidYaw;
    float ratio;
    float kS;
    const float epsilon;
    float setpointPitch, setpointYaw;

public:
    WristSubsystem(
        tap::Drivers *drivers,
        tap::motor::MotorInterface &motorLeft,
        tap::motor::MotorInterface &motorRight,
        tap::algorithms::SmoothPidConfig configPitch,
        tap::algorithms::SmoothPidConfig configYaw,
        float ratio,
        float kS = 0,
        float epsilon = 1);

    float getPitch();

    float getYaw();

    float getSetpointPitch() { return setpointPitch; }

    float getSetpointYaw() { return setpointYaw; }

    void setSetpointPitch(float setpoint) { setpointPitch = setpoint; }

    void setSetpointYaw(float setpoint) { setpointYaw = setpoint; }

    virtual bool atSetpoint();

    virtual void refresh() override;

    virtual void refreshSafeDisconnect() override;
};
}  // namespace engineer
}  // namespace aruwsrc
#endif  // WRIST_SUBSYSTEM_HPP_