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

#ifndef SENTRY_RESPONSE_HANDLER_HPP_
#define SENTRY_RESPONSE_HANDLER_HPP_

#include "tap/communication/serial/ref_serial.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
/**
 * Handles message sent from the sentry and received by other robots.
 */
class SentryResponseHandler
    : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler
{
public:
    SentryResponseHandler(aruwsrc::Drivers &drivers);

    void operator()(
        const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) override final;

    /// @return True if the sentry reports that it is moving, false otherwise.
    inline bool getSentryMoving() const { return this->sentryMoving; }

private:
    aruwsrc::Drivers &drivers;

    bool sentryMoving = true;
};
}  // namespace aruwsrc::communication::serial
#endif  // SENTRY_RESPONSE_HANDLER_HPP_
