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

#ifndef SENTRY_RESPONSE_SUBSYSTEM_HPP_
#define SENTRY_RESPONSE_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

#include "inter_robot_signal_transmitter.hpp"
#include "sentry_strategy_message_types.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
/**
 * Subsystem to broadcast messages to the hero and standard 1 whenever a message is queued.
 * The message is consumed broadcasted
*/
class SentryResponseSubsystem : public tap::control::Subsystem
{
public:
    SentryResponseSubsystem(tap::Drivers *drivers);

    void refresh() override;

    inline mockable void queueResponse(SentryResponseMessageType type)
    {
        sentryResponseTransmitter.queueMessage(type);
    }

private:
    InterRobotSignalMessageTransmitter<
        SentryResponseMessageType,
        static_cast<uint8_t>(SentryResponseMessageType::NUM_MESSAGE_TYPES)>
        sentryResponseTransmitter;
};
}  // namespace aruwsrc::communication::serial

#endif  // SENTRY_RESPONSE_SUBSYSTEM_HPP_
