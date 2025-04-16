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

#ifndef RAW_MOTOR_COMMAND_HPP_
#define RAW_MOTOR_COMMAND_HPP_

#include "tap/communication/serial/remote.hpp"
#include "tap/control/command.hpp"

#include "raw_motor_subsystem.hpp"

namespace aruwsrc::engineer
{

class RawMotorCommand : public tap::control::Command
{
public:
    explicit RawMotorCommand(
        aruwsrc::engineer::RawMotorSubsystem* motor,
        tap::communication::serial::Remote* remote,
        tap::communication::serial::Remote::Channel channel,
        float scalar)
        : motor(motor),
          remote(remote),
          channel(channel),
          scalar(scalar)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(motor));
    }

    inline void initialize() override {}

    inline void execute() override
    {
        motor->setDesiredOutput(remote->getChannel(channel) * scalar);
    }

    void end(bool) override { motor->setDesiredOutput(0); }

    bool isFinished() const override { return false; }

    const char* getName() const override { return "taking this motor raw"; }

private:
    aruwsrc::engineer::RawMotorSubsystem* motor;
    tap::communication::serial::Remote* remote;
    tap::communication::serial::Remote::Channel channel;
    float scalar;

    const float STICK_DEADZONE = 0.05;
};  // class RawMotorCommand

}  // namespace aruwsrc::engineer

#endif  // RAW_MOTOR_COMMAND_HPP_
