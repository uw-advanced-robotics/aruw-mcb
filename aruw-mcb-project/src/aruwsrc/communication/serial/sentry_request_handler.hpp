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

#ifndef SENTRY_REQUEST_HANDLER_HPP_
#define SENTRY_REQUEST_HANDLER_HPP_

#include "tap/communication/serial/ref_serial.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
class SentryRequestHandler
    : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler
{
public:
    using MessageReceivedCallback = void (*)();

    SentryRequestHandler(tap::Drivers *drivers);

    void operator()(
        const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) override final;

    void attachNoStrategyHandler(MessageReceivedCallback callback) { noStrategyHandler = callback; }

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

    void attachHoldFireHandler(MessageReceivedCallback callback) { holdFireHandler = callback; }

    void attachToggleMovementHandler(MessageReceivedCallback callback)
    {
        toggleMovementHandler = callback;
    }

    void attachToggleBeybladeHandler(MessageReceivedCallback callback)
    {
        toggleBeybladeHandler = callback;
    }

private:
    tap::Drivers *drivers;
    MessageReceivedCallback noStrategyHandler = nullptr;
    MessageReceivedCallback goToFriendlyBaseHandler = nullptr;
    MessageReceivedCallback goToEnemyBaseHandler = nullptr;
    MessageReceivedCallback goToSupplierZoneHandler = nullptr;
    MessageReceivedCallback goToEnemySupplierZoneHandler = nullptr;
    MessageReceivedCallback goToCenterPointHandler = nullptr;
    MessageReceivedCallback holdFireHandler = nullptr;
    MessageReceivedCallback toggleMovementHandler = nullptr;
    MessageReceivedCallback toggleBeybladeHandler = nullptr;
};
}  // namespace aruwsrc::communication::serial

#endif  // SENTRY_REQUEST_HANDLER_HPP_
