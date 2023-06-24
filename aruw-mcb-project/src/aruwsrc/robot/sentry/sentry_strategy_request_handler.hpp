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

#ifndef SENTRY_STRATEGY_REQUEST_HANDLER_HPP_
#define SENTRY_STRATEGY_REQUEST_HANDLER_HPP_

#include "tap/communication/serial/ref_serial.hpp"
#include "aruwsrc/control/governor/pause_command_governor.hpp"
#include "sentry_request_message_types.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
// @todo docs
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
class SentryStrategyRequestHandler
    : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler
{
public:
    using MessageReceivedCallback = void (*)();

    SentryStrategyRequestHandler(tap::Drivers *drivers);

    void operator()(
        const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) override final;

    void attachNoStrategyHandler(MessageReceivedCallback callback)
    {
        noStrategyHandler = callback;
    }

    void attachGoToFriendlyBaseHandler(MessageReceivedCallback callback)
    {
        goToFriendlyBaseHandler = callback;
    }

    void attachGoToEnemyBaseHandler(MessageReceivedCallback callback)
    {
        goToEnemyBaseHandler = callback;
    }

    void attachGoToSupplierZoneHandler(MessageReceivedCallback callback)
    {
        goToSupplierZoneHandler = callback;
    }

    void attachGoToEnemySupplierZoneHandler(MessageReceivedCallback callback)
    {
        goToEnemySupplierZoneHandler = callback;
    }

    void attachGoToCenterPointHandler(MessageReceivedCallback callback)
    {
        goToCenterPointHandler = callback;
    }

    void attachHoldFireHandler(MessageReceivedCallback callback)
    {
        holdFireHandler = callback;
    }

    void attachStopMovementHandler(MessageReceivedCallback callback)
    {
        stopMovementHandler = callback;
    }

    void attachStartMovementHandler(MessageReceivedCallback callback)
    {
        startMovementHandler = callback;
    }

    void attachStopBeybladeHandler(MessageReceivedCallback callback)
    {
        stopBeybladeHandler = callback;
    }

    void attachStartBeybladeHandler(MessageReceivedCallback callback)
    {
        startBeybladeHandler = callback;
    }

private:
    tap::Drivers *drivers;
    SentryStrategyRequest lastM = SentryStrategyRequest::NONE;
    MessageReceivedCallback noStrategyHandler = nullptr;
    MessageReceivedCallback goToFriendlyBaseHandler = nullptr;
    MessageReceivedCallback goToEnemyBaseHandler = nullptr;
    MessageReceivedCallback goToSupplierZoneHandler = nullptr;
    MessageReceivedCallback goToEnemySupplierZoneHandler = nullptr;
    MessageReceivedCallback goToCenterPointHandler = nullptr;
    MessageReceivedCallback holdFireHandler = nullptr;
    MessageReceivedCallback stopMovementHandler = nullptr;
    MessageReceivedCallback startMovementHandler = nullptr;
    MessageReceivedCallback stopBeybladeHandler = nullptr;
    MessageReceivedCallback startBeybladeHandler = nullptr;
};

// @todo move
// @todo ad hoc
class SentryHoldFireRequestHandler
    : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler
{
public:
    using MessageReceivedCallback = void (*)();

    SentryHoldFireRequestHandler(PauseCommandGovernor &agitatorPauseGovernor);

    void operator()(
        const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) override final;
private:
    PauseCommandGovernor &agitatorPauseGovernor;
};
}  // namespace aruwsrc::communication::serial

#endif  // SENTRY_STRATEGY_REQUEST_HANDLER_HPP_
