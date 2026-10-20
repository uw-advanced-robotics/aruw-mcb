/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef STICK_TORQUE_COMMAND_HPP_
#define STICK_TORQUE_COMMAND_HPP_

#include <algorithm>

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/robot/motor_tester/motor_subsystem.hpp"

using namespace aruwsrc::motor_tester;
using namespace tap::communication::serial;

namespace aruwsrc::motor_tester
{
class StickTorqueCommand : public tap::control::Command
{
public:
    StickTorqueCommand(
        tap::Drivers* drivers,
        MotorSubsystem* motorSubsystem,
        Remote::Channel channel,
        float sensitivity)
        : drivers(drivers),
          motorSubsystem(motorSubsystem),
          channel(channel),
          SENSITIVITY(sensitivity)
    {
        addSubsystemRequirement(motorSubsystem);
    }

    void execute() override
    {
        float input = drivers->remote.getChannel(channel);
        int scaled_input =
            static_cast<int>((input * SENSITIVITY) * tap::motor::DjiMotor::MAX_OUTPUT_C620);
        motorSubsystem->setDesiredOutput(scaled_input);
    }

    inline void initialize() override {};

    inline bool isFinished() const override { return false; };

    inline void end(bool) override { motorSubsystem->setDesiredOutput(0); };

    inline const char* getName() const override { return "StickTorqueCommand"; };

private:
    tap::Drivers* drivers;
    MotorSubsystem* motorSubsystem;
    Remote::Channel channel;
    const float SENSITIVITY;
};

}  // namespace aruwsrc::motor_tester