/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTINEL_CLIENT_DISPLAY_SUBSYSTEM_HPP_
#define SENTINEL_CLIENT_DISPLAY_SUBSYSTEM_HPP_

#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"

#include "aruwsrc/control/client-display/hud_indicator.hpp"
#include "aruwsrc/control/sentinel/drive/sentinel_auto_drive_comprised_command.hpp"
#include "modm/processing/protothread.hpp"

#include "sentinel_drive_status_hud_indicator.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
class SentinelClientDisplaySubsystem : public tap::control::Subsystem, ::modm::pt::Protothread
{
public:
    SentinelClientDisplaySubsystem(
        aruwsrc::Drivers &drivers,
        aruwsrc::control::sentinel::drive::SentinelAutoDriveComprisedCommand &command);

    void initialize() override;

    void refresh() override;

private:
    aruwsrc::Drivers &drivers;

    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;

    SentinelDriveStatusHudIndicator sentinelDriveStatusHudIndicator;

    bool run();
};
}  // namespace aruwsrc::control::client_display

#endif  // SENTINEL_CLIENT_DISPLAY_SUBSYSTEM_HPP_
