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

#ifndef SENTRY_REQUEST_HANDLER_HPP_
#define SENTRY_REQUEST_HANDLER_HPP_

#include "tap/communication/serial/ref_serial.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
/**
 * Handler that decodes requests made from other robots to the sentry robot. Various robot
 * commands may be sent via the handler. These include the following:
 * - Select new robot: The sentry should target a new robot. If there are no new robots in frame,
 *   doesn't switch target.
 * - Target new quadrant: The sentry will stop targeting and move the turret backwards if it is
 *   pointing forward or forward if it is pointing backwards.
 * - Toggle drive movement: The sentry will switch between driving evasively and driving to the
 *   right of the rail and stopping.
 *
 * Message structure:
 * - byte 1: message type
 * - byte 2-n: message if the message type requires a message
 */
class SentryRequestHandler
    : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler
{
public:
    using MessageReceivedCallback = void (*)();

    SentryRequestHandler(tap::Drivers *drivers);

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
    void attachToggleDriveMovementMessageHandler(MessageReceivedCallback callback)
    {
        toggleDriveMovementMessageHandler = callback;
    }
    void attachPauseProjectileLaunchingMessageHandler(MessageReceivedCallback callback)
    {
        pauseProjectileLaunchingHandler = callback;
    }

private:
    tap::Drivers *drivers;
    MessageReceivedCallback selectNewRobotMessageHandler = nullptr;
    MessageReceivedCallback targetNewQuadrantMessageHandler = nullptr;
    MessageReceivedCallback toggleDriveMovementMessageHandler = nullptr;
    MessageReceivedCallback pauseProjectileLaunchingHandler = nullptr;
};
}  // namespace aruwsrc::communication::serial

#endif  // SENTRY_REQUEST_HANDLER_HPP_
