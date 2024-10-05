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

#include "stick_rpm_command.hpp"

StickRpmCommand::StickRpmCommand(
    MotorSubsystem* subsystem,
    tap::communication::serial::Remote* remote,
    tap::communication::serial::Remote::Channel channel,
    float maxRpm)
    : motorSubsystem(subsystem),
      remote(remote),
      channel(channel),
      maxRpm(maxRpm)
{
    this->addSubsystemRequirement(subsystem);
}

void StickRpmCommand::execute()
{
    float stick = remote->getChannel(this->channel);

    // math for this (unused atm but still): https://www.desmos.com/calculator/ip8m03ugmo
    // float deadenedStick = 0;
    // if (stick > STICK_DEADZONE)
    // {
    //     deadenedStick = (stick - STICK_DEADZONE) / (1 - STICK_DEADZONE);
    // }
    // else if (stick < -STICK_DEADZONE)
    // {
    //     deadenedStick = (stick + STICK_DEADZONE) / (1 - STICK_DEADZONE);
    // }
    motorSubsystem->setDesiredRPM(maxRpm * stick);
}

void StickRpmCommand::end(bool) { motorSubsystem->stop(); }