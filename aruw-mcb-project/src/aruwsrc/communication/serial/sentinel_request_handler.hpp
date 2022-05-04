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

#ifndef SENTINEL_REQUEST_HANDLER_HPP_
#define SENTINEL_REQUEST_HANDLER_HPP_

#include "tap/communication/serial/ref_serial.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
/**
 * Handler that decodes requests made from other robots to the sentinel robot. Various robot
 * commands may be sent via the handler. These include the following:
 * - Select new robot: The sentinel should target a new robot. If there are no new robots in frame,
 *   doesn't switch target.
 * - Target new quadrant: The sentinel will stop targeting and move the turret backwards if it is
 *   pointing forward or forward if it is pointing backwards.
 *
 * Message structure:
 * - byte 1: message type
 * - byte 2-n: message if the message type requires a message
 */
class SentinelRequestHandler
    : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler
{
public:
    using MessageReceivedCallback = void (*)();

    SentinelRequestHandler(aruwsrc::Drivers *drivers);

    void operator()(
        const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) override final;

    void attachSelectNewRobotMessageHandler(MessageReceivedCallback callback)
    {
        selectNewRobotMessageHandler = callback;
    }
    void attachTargetNewQuadrantMessageHandler(MessageReceivedCallback callback)
    {
        targetNewQuadrantMessageHandler = callback;
    }

private:
    aruwsrc::Drivers *drivers;
    MessageReceivedCallback selectNewRobotMessageHandler = nullptr;
    MessageReceivedCallback targetNewQuadrantMessageHandler = nullptr;
};
}  // namespace aruwsrc::communication::serial

#endif  // SENTINEL_REQUEST_HANDLER_HPP_
