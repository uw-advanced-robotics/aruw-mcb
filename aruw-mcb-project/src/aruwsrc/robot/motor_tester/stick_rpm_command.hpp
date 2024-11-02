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

#ifndef STICK_RPM_COMMAND_HPP_
#define STICK_RPM_COMMAND_HPP_

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "motor_subsystem.hpp"

using namespace aruwsrc::motor_tester;

class StickRpmCommand : public tap::control::Command
{
public:
    explicit StickRpmCommand(
        MotorSubsystem* subsystem,
        tap::communication::serial::Remote* remote,
        tap::communication::serial::Remote::Channel channel,
        float maxRpm);

    void initialize() override {}

    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "stick rpm"; }

private:
    MotorSubsystem* motorSubsystem;
    tap::communication::serial::Remote* remote;
    tap::communication::serial::Remote::Channel channel;
    float maxRpm;

    const float STICK_DEADZONE = 0.05;
};  // class StickRpmCommand

#endif  // STICK_RPM_COMMAND_HPP_
