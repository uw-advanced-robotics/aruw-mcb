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

#include "inter_robot_signal_receiver.hpp"

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/architecture/timeout.hpp"

namespace aruwsrc::communication::serial
{

enum class SentryStrategy : uint8_t
{
    NONE = 0,
    GO_TO_FRIENDLY_BASE,
    GO_TO_ENEMY_BASE,
    GO_TO_FRIENDLY_SUPPLIER_ZONE,
    GO_TO_ENEMY_SUPPLIER_ZONE,
    GO_TO_CENTER_POINT,
};

/**
 * Handles message sent from the sentry and received by other robots.
 */
class SentryResponseHandler
    : public tap::communication::serial::RefSerial::RobotToRobotMessageHandler
{
public:
    SentryResponseHandler(tap::Drivers &drivers);

    void operator()(
        const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) override final;

    inline bool getSentryMovementEnabled() const { return this->sentryMovementEnabled; }
    inline bool getSentryBeybladeEnabled() const { return this->sentryBeybladeEnabled; }

    inline SentryStrategy getSentryStrategy() const { return this->sentryStrategy; }

    // @todo fix once taproot MR !240 is merged
    // inline uint32_t getHoldFireTimeRemainingSec() const { return int(this->holdFireTimer.timeRemaining() / 1000); }
    inline uint32_t getHoldFireTimeRemainingSec() const { return 0; }

private:
    tap::Drivers &drivers;

    InterRobotSignalReceiver<SentryResponseMessageType, static_cast<uint8_t>(SentryResponseMessageType::NUM_MESSAGE_TYPES)> signalReceiver;

    bool sentryMovementEnabled = true;
    bool sentryBeybladeEnabled = true;

    tap::arch::MilliTimeout holdFireTimer;
    SentryStrategy sentryStrategy = SentryStrategy::NONE;
};
}  // namespace aruwsrc::communication::serial
#endif  // SENTRY_RESPONSE_HANDLER_HPP_
