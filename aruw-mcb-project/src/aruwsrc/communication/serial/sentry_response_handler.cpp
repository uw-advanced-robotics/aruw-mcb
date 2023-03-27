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

#include "sentry_response_handler.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

namespace aruwsrc::communication::serial
{
SentryResponseHandler::SentryResponseHandler(tap::Drivers &drivers) : drivers(drivers) {}

void SentryResponseHandler::operator()(
    const tap::communication::serial::DJISerial::ReceivedSerialMessage &message)
{
    if (message.header.dataLength !=
        sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader) + 1)
    {
        RAISE_ERROR((&drivers), "message length incorrect");
        return;
    }

    this->sentryMoving = static_cast<bool>(
        message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);
}
}  // namespace aruwsrc::communication::serial
