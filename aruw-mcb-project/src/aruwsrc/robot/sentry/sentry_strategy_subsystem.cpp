// /*
//  * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

// #include "sentry_response_subsystem.hpp"

// #include "tap/drivers.hpp"

// namespace aruwsrc::communication::serial
// {
// SentryStrategySubsystem::SentryStrategySubsystem(tap::Drivers& drivers)
//     : tap::control::Subsystem(drivers),
//       refSerialTransmitter(drivers.refSerialTransmitter)
// {
// }

// void SentryStrategySubsystem::refresh() { 
//     PT_BEGIN();

//     while (true)
//     {
//         getNextMessageToSend();

//         if ((queuedMessageType & (1 << static_cast<uint8_t>(lastSentMessage))) != 0)
//         {
//             *reinterpret_cast<uint16_t*>(this->robotToRobotMessage.dataAndCrc16) = static_cast<uint8_t>(lastSentMessage);

//             // TODO configure rest of message if required by message type, currently this is not
//             // necessary

//             PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
//                 &robotToRobotMessage,
//                 SENTRY_REQUEST_ROBOT_ID,
//                 drivers->refSerial.getRobotIdBasedOnCurrentRobotTeam(
//                     RefSerialData::RobotId::BLUE_SENTINEL),
//                 2));

//             queuedMessageType &= ~(1 << static_cast<uint8_t>(lastSentMessage));
//         }
//         else
//         {
//             PT_YIELD();
//         }
//     }

//     PT_END();
// }
// }  // namespace aruwsrc::communication::serial