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

#ifndef DAMIAO_STICK_VELOCITY_COMMAND_HPP_
#define DAMIAO_STICK_VELOCITY_COMMAND_HPP_

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/control/motor/damiao_subsystem.hpp"

namespace aruwsrc::motor_tester
{
class DamiaoStickVelocityCommand : public tap::control::Command
{
public:
    DamiaoStickVelocityCommand(
        control::motor::DamiaoSubsystem& subsystem,
        tap::communication::serial::Remote& remote,
        tap::communication::serial::Remote::Channel channel,
        float maxVelocityRadPerSec);

    void initialize() override {}
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override { return false; }
    const char* getName() const override { return "Damiao stick velocity"; }

private:
    control::motor::DamiaoSubsystem& subsystem;
    tap::communication::serial::Remote& remote;
    tap::communication::serial::Remote::Channel channel;

    float maxVelocityRadPerSec;
    float STICK_DEADZONE = 0.05f;
    float KD = 0.1f;
    volatile float DEBUG_wrappedPosition = 0.0f;
    volatile float DEBUG_unwrappedPosition = 0.0f;
};
}  // namespace aruwsrc::motor_tester

#endif  // DAMIAO_STICK_VELOCITY_COMMAND_HPP_
