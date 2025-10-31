/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MOTOR_SUBSYSTEM_HPP_
#define MOTOR_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "tap/motor/motor_interface.hpp"
#include "tap/motor/dji_motor.hpp"

#include "tap/algorithms/smooth_pid.hpp"
#include <tap/algorithms/wrapped_float.hpp>

namespace aruwsrc::motor_tester
{
class MotorSubsystem : public tap::control::Subsystem
{
public:
    MotorSubsystem(tap::Drivers* drivers, tap::motor::MotorInterface* motorInterface, tap::algorithms::SmoothPidConfig config);

    const char* getName() const override { return "Motor DJI Edu"; }

    void refreshSafeDisconnect() override;

    void initialize() override;
    void refresh() override;

    // void setDesiredOutput(int32_t value);
    // int32_t getDesiredOutput();

    void setDesiredPosition(tap::algorithms::WrappedFloat radians);
    tap::algorithms::WrappedFloat getDesiredPosition() const;

    bool isOnline() const;

public:
    // int32_t proportion;
private:
    tap::motor::MotorInterface* motorInterface;
    // int32_t desiredOutput;
    // float desiredPosition;
    tap::algorithms::WrappedFloat desiredPosition;

    tap::algorithms::SmoothPid pid;

    uint32_t prevTime;

    
};

}  // namespace aruwsrc::control::motor

#endif  // MOTOR_SUBSYSTEM_HPP_