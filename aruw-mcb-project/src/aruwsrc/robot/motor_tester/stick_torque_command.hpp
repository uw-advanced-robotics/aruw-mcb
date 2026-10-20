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

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/robot/motor_tester/motor_subsystem.hpp"

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
          sensitivity(sensitivity)
    {
        addSubsystemRequirement(motorSubsystem);
    }

    void execute() override
    {
        const float input = drivers->remote.getChannel(channel);
        position += input * sensitivity;
        motorSubsystem->setDesiredPosition(position);
    }

    void initialize() override { position = motorSubsystem->getDesiredPosition(); }

    bool isFinished() const override { return false; }

    void end(bool) override { motorSubsystem->setDesiredPosition(position); }

    const char* getName() const override { return "stick torque"; }

private:
    tap::Drivers* drivers;
    MotorSubsystem* motorSubsystem;
    Remote::Channel channel;
    const float sensitivity;
    float position = 0.0f;
};

}  // namespace aruwsrc::motor_tester

#endif  // STICK_TORQUE_COMMAND_HPP_
