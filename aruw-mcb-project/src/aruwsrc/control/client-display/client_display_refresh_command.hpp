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

#ifndef CLIENT_DISPLAY_REFRESH_COMMAND_HPP_
#define CLIENT_DISPLAY_REFRESH_COMMAND_HPP_

#include <vector>

#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "indicators/hud_indicator.hpp"
#include "modm/processing/protothread.hpp"

#include "client_display_subsystem.hpp"

namespace aruwsrc::control::client_display
{
using namespace tap::communication::serial;

class ClientDisplaySubsystem;

/**
 * A No-Op commmand that utilizes the ClientDisplaySubsystem to refresh the client display.
 */
class ClientDisplayCommand : public tap::control::Command
{
public:
    ClientDisplayCommand(ClientDisplaySubsystem &clientDisplay) : Command()
    {
        addSubsystemRequirement(&clientDisplay);
    }

    const char *getName() const override { return "Client Display Refresh"; }

    void initialize() override {}

    void execute() override {};

    void end(bool) override {}

    bool isFinished() const override { return true; }
};
}  // namespace aruwsrc::control::client_display

#endif  // CLIENT_DISPLAY_REFRESH_COMMAND_HPP_
