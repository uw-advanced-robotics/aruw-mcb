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

#include "sentry_motion_strategy_messages.hpp"

namespace aruwsrc::communication::serial
{
SentryRequestHandler::SentryRequestHandler(tap::Drivers *drivers) : drivers(drivers) {}

void SentryRequestHandler::operator()(
    const tap::communication::serial::DJISerial::ReceivedSerialMessage &message)
{
    // The message type we sent came directly after the interactive header
    SentryMotionStrategyMessages type = static_cast<SentryMotionStrategyMessages>(
        message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);

    switch (type)
    {
        case SentryMotionStrategyMessages::NONE:
            if (noStrategyHandler != nullptr)
            {
                noStrategyHandler(); // @TODO: make sure we know message signature, set this up in a better way with message types
            }
            break;
        case SentryMotionStrategyMessages::GO_TO_FRIENDLY_BASE:
            if (goToFriendlyBaseHandler != nullptr)
            {
                goToFriendlyBaseHandler(); // @TODO: make sure we know message signature, set this up in a better way with message types
            }
            break;
        case SentryMotionStrategyMessages::GO_TO_ENEMY_BASE:
            if (goToEnemyBaseHandler != nullptr)
            {
                goToEnemyBaseHandler(); // @TODO: make sure we know message signature, set this up in a better way with message types
            }
            break;
        case SentryMotionStrategyMessages::GO_TO_SUPPLIER_ZONE:
            if (goToSupplierZoneHandler != nullptr)
            {
                goToSupplierZoneHandler(); // @TODO: make sure we know message signature, set this up in a better way with message types
            }
            break;
        default:
            RAISE_ERROR(drivers, "invalid message sentry request message type");
            break;
    }
}
}  // namespace aruwsrc::communication::serial
