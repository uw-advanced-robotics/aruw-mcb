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

#ifndef SENTINEL_RESPONSE_HANDLER_HPP_
#define SENTINEL_RESPONSE_HANDLER_HPP_

#include "tap/communication/serial/ref_serial.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
/**
 * Handles message sent from the sentinel and received by other robots.
 */
class SentinelResponseHandler
    : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler
{
public:
    SentinelResponseHandler(aruwsrc::Drivers &drivers);

    void operator()(
        const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) override final;

    /// @return True if the sentinel reports that it is moving, false otherwise.
    inline bool getSentinelMoving() const { return this->sentinelMoving; }

private:
    aruwsrc::Drivers &drivers;

    bool sentinelMoving = true;
};
}  // namespace aruwsrc::communication::serial
#endif  // SENTINEL_RESPONSE_HANDLER_HPP_
