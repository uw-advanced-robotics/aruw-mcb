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

#ifndef CLIENT_DISPLAY_SUBSYSTEM_HPP_
#define CLIENT_DISPLAY_SUBSYSTEM_HPP_

#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
/**
 * A placeholder subsystem for running the client display command
 */
class ClientDisplaySubsystem : public tap::control::Subsystem
{
public:
    ClientDisplaySubsystem(tap::Drivers* drivers);
    virtual ~ClientDisplaySubsystem() {}

    void refreshSafeDisconnect() override {}

    const char* getName() override { return "client display"; }
};
}  // namespace aruwsrc::control::client_display

#endif  // CLIENT_DISPLAY_SUBSYSTEM_HPP_
