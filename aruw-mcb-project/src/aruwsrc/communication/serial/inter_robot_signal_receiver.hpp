/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef INTER_ROBOT_SIGNAL_RECEIVER_HPP_
#define INTER_ROBOT_SIGNAL_RECEIVER_HPP_

#ifdef ENV_UNIT_TESTS
#include "tap/mock/ref_serial_transmitter_mock.hpp"
#else
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#endif
#include "tap/communication/serial/ref_serial_data.hpp"

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include <type_traits>

namespace aruwsrc
{
class Drivers;
}

// template<typename MSG_TYPE_ENUM>
// concept Enumable = requires
// {
//     MSG_TYPE
// }

namespace aruwsrc::communication::serial
{
template<typename MSG_TYPE_ENUM, uint8_t NUM_MSG_TYPES>
class InterRobotSignalReceiver
{
    static_assert(std::is_enum_v<MSG_TYPE_ENUM>, "MSG_TYPE_ENUM must be an enum.");
    // static_assert(std::is_same_v<uint8_t, std::underlying_type_t<MSG_TYPE_ENUM>>, "Message type enum must be uint8_t.");
    static_assert(NUM_MSG_TYPES <= 32, "Only 32 message types maximum allowed.");
    static_assert(NUM_MSG_TYPES >= 1, "There must at least be 1 message type.");
public:
    inline InterRobotSignalReceiver(tap::Drivers *drivers) : drivers(drivers)
    {
    }

    inline MSG_TYPE_ENUM parse(const tap::communication::serial::DJISerial::ReceivedSerialMessage &message) const
    {
        if (message.header.dataLength !=
            sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader) + sizeof(MSG_TYPE_ENUM))
        {
            RAISE_ERROR((drivers), "message length incorrect");
        }
        // In a robot-to-robot message, the message data has an InteractiveHeader and then the actual contents
        // The actual contents is just a signal type
        return static_cast<MSG_TYPE_ENUM>(message.data[sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader)]);
    }
private:
    tap::Drivers *drivers;
};
}  // namespace aruwsrc::communication::serial

#endif  // INTER_ROBOT_SIGNAL_RECEIVER_HPP_
