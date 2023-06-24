// /*
//  * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
//  *
//  * This file is part of aruw-mcb.
//  *
//  * aruw-mcb is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * aruw-mcb is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
//  */

// #ifndef ROBOT_TRANSMITTER_HPP_
// #define ROBOT_TRANSMITTER_HPP_

// #ifdef ENV_UNIT_TESTS
// #include "tap/mock/ref_serial_transmitter_mock.hpp"
// #else
// #include "tap/communication/serial/ref_serial_transmitter.hpp"
// #endif

// #include "modm/architecture/interface/register.hpp"
// #include "modm/processing/protothread.hpp"

// #include "sentry_request_message_types.hpp"

// #include <type_traits>
// #include <optional>

// namespace aruwsrc
// {
// class Drivers;
// }
// namespace aruwsrc::communication::serial
// {
// template <typename MessageTypeEnum>
// class InterRobotTransmitter : public modm::pt::Protothread
// {
//     static_assert(std::is_enum<MessageTypeEnum>>::value)
//     // @todo static assert enum of uint type
// public:
//     InterRobotTransmitter(tap::Drivers *drivers, RobotId receiverRobotID);

//     bool send();

//     void queueRequest(MessageTypeEnum type);
// private:
//     tap::Drivers *drivers;

// #ifdef ENV_UNIT_TESTS
// public:
//     tap::mock::RefSerialTransmitterMock refSerialTransmitter;

// private:
// #else
//     tap::communication::serial::RefSerialTransmitter refSerialTransmitter;
// #endif
//     std::optional<MessageTypeEnum> lastSentMessage = std::nullopt;
//     uint32_t queuedMessageBitmask{};
//     tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

//     inline void getNextMessageToSend()
//     {
//         // either no queued messages or lastSentMessage is the only message to send, return
//         // w/o trying to find a new message
//         if (queuedMessageBitmask == 0)
//         {
//             return;
//         }

//         if ((lastSentMessage.has_value() != queuedMessageBitmask & ~(1 << static_cast<uint8_t>(lastSentMessage.value()))) == 0)
//         {
//             return;
//         }

//         // otherwise, iterate through message types until you find one that is queued
//         auto nextMessageType = [](MessageTypeEnum type) {
//             return static_cast<MessageTypeEnum>(
//                 (static_cast<uint8_t>(type) + 1) %
//                 static_cast<uint8_t>(MessageTypeEnum::NUM_MESSAGE_TYPES));
//         };

//         lastSentMessage = nextMessageType(lastSentMessage);

//         while ((queuedMessageBitmask & (1 << static_cast<uint8_t>(lastSentMessage))) == 0)
//         {
//             lastSentMessage = nextMessageType(lastSentMessage);
//         }
//     }
// };
// }  // namespace aruwsrc::communication::serial

// #endif  // SENTRY_REQUEST_TRANSMITTER_HPP_