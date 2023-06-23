/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "drone_telemetry_handler.hpp"

namespace aruwsrc::drone
{
DroneTelemetryHandler::DroneTelemetryHandler(
    tap::Drivers* drivers,
    tap::communication::serial::Uart::UartPort port)
    : MavlinkReceiver(drivers, port),
      setHomeCommand(),
      localPositionNed()
{
}

void DroneTelemetryHandler::messageReceiveCallback(const ReceivedMavlinkMessage& completeMessage)
{
    gotAMessage = true;
    if (completeMessage.header.messageId == LOCAL_POSITION_NED_MSG_ID)
    {
        memcpy(&localPositionNed, &completeMessage.data, sizeof(localPositionNed));
    }
}

LocalPositionNed* DroneTelemetryHandler::getLocalPositionNed() { return &localPositionNed; }

void DroneTelemetryHandler::setHomePosition()
{
    setHomeCommand.header.componentId = 1;
    setHomeCommand.header.systemId = 2;  // Needs to be not 0 or 1 cuz 0 invalid, 1 is drone
    setHomeCommand.header.messageId = COMMAND_INT_MSG_ID;
    
    memcpy(setHomeCommand.data, &cmd, sizeof(cmd));
    
    drivers->uart.write(port, reinterpret_cast<uint8_t*>(&setHomeCommand), sizeof(setHomeCommand));
}

}  // namespace aruwsrc::drone
