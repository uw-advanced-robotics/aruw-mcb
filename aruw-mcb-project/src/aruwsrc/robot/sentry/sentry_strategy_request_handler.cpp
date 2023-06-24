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

#include "sentry_strategy_request_handler.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "sentry_request_message_types.hpp"

namespace aruwsrc::communication::serial
{
SentryStrategyRequestHandler::SentryStrategyRequestHandler(tap::Drivers *drivers) : drivers(drivers) {}

void SentryStrategyRequestHandler::operator()(
    const tap::communication::serial::DJISerial::ReceivedSerialMessage &message)
{
    // The message type we sent came directly after the interactive header
    SentryStrategyRequest type = static_cast<SentryStrategyRequest>(
        message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);

    lastM = type;
    switch (type)
    {
        case SentryStrategyRequest::NONE:
            if (noStrategyHandler != nullptr)
            {
                noStrategyHandler(); 
            }
            break;
        case SentryStrategyRequest::GO_TO_FRIENDLY_BASE:
            if (goToFriendlyBaseHandler != nullptr)
            {
                goToFriendlyBaseHandler();
            }
            break;
        case SentryStrategyRequest::GO_TO_ENEMY_BASE:
            if (goToEnemyBaseHandler != nullptr)
            {
                goToEnemyBaseHandler(); 
            }
            break;
        case SentryStrategyRequest::GO_TO_SUPPLIER_ZONE:
            if (goToSupplierZoneHandler != nullptr)
            {
                goToSupplierZoneHandler(); 
            }
            break;
        case SentryStrategyRequest::GO_TO_ENEMY_SUPPLIER_ZONE:
            if (goToEnemySupplierZoneHandler != nullptr)
            {
                goToEnemySupplierZoneHandler();
            }
            break;
        case SentryStrategyRequest::GO_TO_CENTER_POINT:
            if (goToCenterPointHandler != nullptr)
            {
                goToCenterPointHandler();
            }
            break;
        case SentryStrategyRequest::HOLD_FIRE:
            if (holdFireHandler != nullptr)
            {
                holdFireHandler();
            }
            break;
        case SentryStrategyRequest::STOP_MOVEMENT:
            if (stopMovementHandler != nullptr)
            {
                stopMovementHandler(); 
            }
            break;
        case SentryStrategyRequest::START_MOVEMENT:
            if (startMovementHandler != nullptr)
            {
                startMovementHandler();
            }
            break;
        case SentryStrategyRequest::STOP_BEYBLADE:
            if (stopBeybladeHandler != nullptr)
            {
                stopBeybladeHandler(); 
            }
            break;
        case SentryStrategyRequest::START_BEYBLADE:
            if (startBeybladeHandler != nullptr)
            {
                startBeybladeHandler();
            }
            break;
        default:
            RAISE_ERROR(drivers, "invalid message sentry request message type");
            break;
    }
}

}  // namespace aruwsrc::communication::serial
