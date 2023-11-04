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

#include "sentry_response_handler.hpp"
#include "sentry_strategy_message_types.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

namespace aruwsrc::communication::serial
{
SentryResponseHandler::SentryResponseHandler(tap::Drivers &drivers) : drivers(drivers) {}

void SentryResponseHandler::operator()(
    const tap::communication::serial::DJISerial::ReceivedSerialMessage &message)
{
    SentryResponseMessageType type = signalReciver.parse(message);

    switch (type)
    {
        case SentryResponseMessageType::MOVEMENT_ENABLED:
            this->sentryMovementEnabled = true;
            break;
        case SentryResponseMessageType::MOVEMENT_DISABLED:
            this->sentryMovementEnabled = false;
            break;
        case SentryResponseMessageType::BEYBLADE_ENABLED:
            this->sentryBeybladeEnabled = true;
            break;
        case SentryResponseMessageType::BEYBLADE_DISABLED:
            this->sentryBeybladeEnabled = false;
            break;
        case SentryResponseMessageType::HOLD_FIRE:
            this->holdFireTimer.restart(10000);
            break;
        default:
            this->sentryStrategy = static_cast<SentryStrategy>(type);
    }
}
}  // namespace aruwsrc::communication::serial
