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

#include "sentinel_client_display_subsystem.hpp"

#include "aruwsrc/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::control::client_display
{
SentinelClientDisplaySubsystem::SentinelClientDisplaySubsystem(
    aruwsrc::Drivers &drivers,
    aruwsrc::control::sentinel::drive::SentinelAutoDriveComprisedCommand &command)
    : tap::control::Subsystem(&drivers),
      drivers(drivers),
      refSerialTransmitter(&drivers),
      sentinelDriveStatusHudIndicator(drivers, refSerialTransmitter, command)
{
}

void SentinelClientDisplaySubsystem::initialize()
{
    this->sentinelDriveStatusHudIndicator.initialize();
}

void SentinelClientDisplaySubsystem::refresh() { this->run(); }

bool SentinelClientDisplaySubsystem::run()
{
    PT_BEGIN();

    PT_WAIT_UNTIL(this->drivers.refSerial.getRefSerialReceivingData());

    PT_CALL(this->sentinelDriveStatusHudIndicator.sendInitialGraphics());

    while (true)
    {
        PT_CALL(this->sentinelDriveStatusHudIndicator.update());

        PT_YIELD();
    }

    PT_END();
}
}  // namespace aruwsrc::control::client_display
