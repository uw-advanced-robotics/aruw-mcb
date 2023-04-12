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

#include "sentry_request_handler.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "sentry_request_message_types.hpp"

namespace aruwsrc::communication::serial
{
SentryRequestHandler::SentryRequestHandler(tap::Drivers *drivers) : drivers(drivers) {}

void SentryRequestHandler::operator()(
    const tap::communication::serial::DJISerial::ReceivedSerialMessage &message)
{
    // The message type we sent came directly after the interactive header
    SentryRequestMessageType type = static_cast<SentryRequestMessageType>(
        message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);

    switch (type)
    {
        case SentryRequestMessageType::SELECT_NEW_ROBOT:
            if (selectNewRobotMessageHandler != nullptr)
            {
                selectNewRobotMessageHandler();
            }
            break;
        case SentryRequestMessageType::TARGET_NEW_QUADRANT:
            if (targetNewQuadrantMessageHandler != nullptr)
            {
                targetNewQuadrantMessageHandler();
            }
            break;
        case SentryRequestMessageType::TOGGLE_DRIVE_MOVEMENT:
            if (toggleDriveMovementMessageHandler != nullptr)
            {
                toggleDriveMovementMessageHandler();
            }
            break;
        case SentryRequestMessageType::PAUSE_PROJECTILE_LAUNCHING:
            if (pauseProjectileLaunchingHandler != nullptr)
            {
                pauseProjectileLaunchingHandler();
            }
            break;
        default:
            RAISE_ERROR(drivers, "invalid message sentry request message type");
            break;
    }
}
}  // namespace aruwsrc::communication::serial
