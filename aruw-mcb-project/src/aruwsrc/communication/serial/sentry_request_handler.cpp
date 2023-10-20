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

#include "sentry_request_handler.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "sentry_strategy_message_types.hpp"

namespace aruwsrc::communication::serial
{
SentryRequestHandler::SentryRequestHandler(tap::Drivers *drivers) : drivers(drivers) {}

void SentryRequestHandler::operator()(
    const tap::communication::serial::DJISerial::ReceivedSerialMessage &message)
{
    // The message type we sent came directly after the interactive header
    // SentryRequestMessageType type = signalReceiver.parse(message);
    SentryRequestMessageType type = static_cast<SentryRequestMessageType>(
        message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);

    switch (type)
    {
        case SentryRequestMessageType::NONE:
            if (noStrategyHandler != nullptr)
            {
                noStrategyHandler();
            }
            break;
        case SentryRequestMessageType::GO_TO_FRIENDLY_BASE:
            if (goToFriendlyBaseHandler != nullptr)
            {
                goToFriendlyBaseHandler();
            }
            break;
        case SentryRequestMessageType::GO_TO_ENEMY_BASE:
            if (goToEnemyBaseHandler != nullptr)
            {
                goToEnemyBaseHandler();
            }
            break;
        case SentryRequestMessageType::GO_TO_FRIENDLY_SUPPLIER_ZONE:
            if (goToSupplierZoneHandler != nullptr)
            {
                goToSupplierZoneHandler();
            }
            break;
        case SentryRequestMessageType::GO_TO_ENEMY_SUPPLIER_ZONE:
            if (goToEnemySupplierZoneHandler != nullptr)
            {
                goToEnemySupplierZoneHandler();
            }
            break;
        case SentryRequestMessageType::GO_TO_CENTER_POINT:
            if (goToCenterPointHandler != nullptr)
            {
                goToCenterPointHandler();
            }
            break;
        case SentryRequestMessageType::HOLD_FIRE:
            if (holdFireHandler != nullptr)
            {
                holdFireHandler();
            }
            break;
        case SentryRequestMessageType::TOGGLE_MOVEMENT:
            if (toggleMovementHandler != nullptr)
            {
                toggleMovementHandler();
            }
            break;
        case SentryRequestMessageType::TOGGLE_BEYBLADE:
            if (toggleBeybladeHandler != nullptr)
            {
                toggleBeybladeHandler();
            }
            break;
        default:
            RAISE_ERROR(drivers, "invalid sentry request message type");
            break;
    }
}

}  // namespace aruwsrc::communication::serial
