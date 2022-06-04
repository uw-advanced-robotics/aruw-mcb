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

#include "sentinel_request_handler.hpp"

#include "tap/errors/create_errors.hpp"

#include "aruwsrc/drivers.hpp"

#include "sentinel_request_message_types.hpp"

namespace aruwsrc::communication::serial
{
SentinelRequestHandler::SentinelRequestHandler(aruwsrc::Drivers *drivers) : drivers(drivers) {}

void SentinelRequestHandler::operator()(
    const tap::communication::serial::DJISerial::ReceivedSerialMessage &message)
{
    // The message type we sent came directly after the interactive header
    SentinelRequestMessageType type = static_cast<SentinelRequestMessageType>(
        message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);

    switch (type)
    {
        case SentinelRequestMessageType::SELECT_NEW_ROBOT:
            if (selectNewRobotMessageHandler != nullptr) selectNewRobotMessageHandler();
            break;
        case SentinelRequestMessageType::TARGET_NEW_QUADRANT:
            if (targetNewQuadrantMessageHandler != nullptr) targetNewQuadrantMessageHandler();
            break;
        case SentinelRequestMessageType::TOGGLE_DRIVE_MOVEMENT:
            if (toggleDriveMovementMessageHandler != nullptr) toggleDriveMovementMessageHandler();
            break;
        default:
            RAISE_ERROR(drivers, "invalid message sentinel request message type");
            break;
    }
}
}  // namespace aruwsrc::communication::serial
