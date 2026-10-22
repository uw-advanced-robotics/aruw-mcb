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

#include "virtual_dji_motor_tx_handler.hpp"

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

namespace aruwsrc::communication::mcb_lite::motor
{
VirtualDJIMotorTxHandler::VirtualDJIMotorTxHandler(tap::Drivers* drivers)
    : DjiMotorTxHandler(drivers),
      can1MessageLowSend(),
      can1MessageHighSend(),
      can1Message6020CurrentSend(),
      can2MessageLowSend(),
      can2MessageHighSend(),
      can2Message6020CurrentSend()
{
}

void VirtualDJIMotorTxHandler::encodeAndSendCanData()
{
    // set up new can messages to be sent via CAN bus 1 and 2
    modm::can::Message can1MessageLow(
        CAN_DJI_LOW_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can1MessageHigh(
        CAN_DJI_HIGH_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can1Message6020Current(
        CAN_DJI_6020_CURRENT_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can2MessageLow(
        CAN_DJI_LOW_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can2MessageHigh(
        CAN_DJI_HIGH_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);
    modm::can::Message can2Message6020Current(
        CAN_DJI_6020_CURRENT_IDENTIFIER,
        CAN_DJI_MESSAGE_SEND_LENGTH,
        0,
        false);

    bool can1ValidMotorMessageLow = false;
    bool can1ValidMotorMessageHigh = false;
    bool can1ValidMotorMessage6020Current = false;
    bool can2ValidMotorMessageLow = false;
    bool can2ValidMotorMessageHigh = false;
    bool can2ValidMotorMessage6020Current = false;

    serializeMotorStoreSendData(
        can1MotorStore,
        &can1MessageLow,
        &can1MessageHigh,
        &can1Message6020Current,
        &can1ValidMotorMessageLow,
        &can1ValidMotorMessageHigh,
        &can1ValidMotorMessage6020Current);

    serializeMotorStoreSendData(
        can2MotorStore,
        &can2MessageLow,
        &can2MessageHigh,
        &can2Message6020Current,
        &can2ValidMotorMessageLow,
        &can2ValidMotorMessageHigh,
        &can2ValidMotorMessage6020Current);

    if (can1ValidMotorMessageLow)
    {
        memcpy(can1MessageLowSend.data, &can1MessageLow, sizeof(modm::can::Message));
        can1MessageLowSend.messageType = 0;
        can1MessageLowSend.setCRC16();
    }
    if (can1ValidMotorMessageHigh)
    {
        memcpy(can1MessageHighSend.data, &can1MessageHigh, sizeof(modm::can::Message));
        can1MessageHighSend.messageType = 0;
        can1MessageHighSend.setCRC16();
    }
    if (can1ValidMotorMessage6020Current)
    {
        memcpy(
            can1Message6020CurrentSend.data,
            &can1Message6020Current,
            sizeof(modm::can::Message));
        can1Message6020CurrentSend.messageType = 0;
        can1Message6020CurrentSend.setCRC16();
    }
    if (can2ValidMotorMessageLow)
    {
        memcpy(can2MessageLowSend.data, &can2MessageLow, sizeof(modm::can::Message));
        can2MessageLowSend.messageType = 1;
        can2MessageLowSend.setCRC16();
    }
    if (can2ValidMotorMessageHigh)
    {
        memcpy(can2MessageHighSend.data, &can2MessageHigh, sizeof(modm::can::Message));
        can2MessageHighSend.messageType = 1;
        can2MessageHighSend.setCRC16();
    }
    if (can2ValidMotorMessage6020Current)
    {
        memcpy(
            can2Message6020CurrentSend.data,
            &can2Message6020Current,
            sizeof(modm::can::Message));
        can2Message6020CurrentSend.messageType = 1;
        can2Message6020CurrentSend.setCRC16();
    }
}

}  // namespace aruwsrc::communication::mcb_lite::motor
