/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "virtual_can_bus.hpp"

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::virtualMCB
{
VirtualCanBus::VirtualCanBus(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port)
    : DJISerial(drivers, port),
      port(port),
      currentIMUData(),
      currentCurrentSensorData()
{
}

IMUMessage& VirtualCanBus::getIMUMessage() { return currentIMUData; }

CurrentSensorMessage& VirtualCanBus::getCurrentSensorMessage() { return currentCurrentSensorData; }

void VirtualCanBus::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
{
    switch (completeMessage.messageType)
    {
        {
            case MessageTypes::CANBUS1_MESSAGE:
                processCanMessage(completeMessage, tap::can::CanBus::CAN_BUS1);
                break;
            case MessageTypes::CANBUS2_MESSAGE:
                processCanMessage(completeMessage, tap::can::CanBus::CAN_BUS2);
                break;
            case MessageTypes::IMU_MESSAGE:
                processIMUMessage(completeMessage);
                break;
            case MessageTypes::GPIO_MESSAGE:
                break;
            default:
                break;
        }
    }
}

void VirtualCanBus::processCanMessage(
    const ReceivedSerialMessage& completeMessage,
    tap::can::CanBus canbus)
{
    // Read the message
    modm::can::Message msg;
    memcpy(&msg, completeMessage.data, sizeof(modm::can::Message));

    // Push the message to the correct queue
    if (canbus == tap::can::CanBus::CAN_BUS1)
    {
        CAN1_queue.push(msg);
    }
    else
    {
        CAN2_queue.push(msg);
    }
}

void VirtualCanBus::processIMUMessage(const ReceivedSerialMessage& completeMessage)
{
    // Read the message
    memcpy(&currentIMUData, completeMessage.data, sizeof(IMUMessage));
}

void VirtualCanBus::processCurrentSensorMessage(const ReceivedSerialMessage& completeMessage)
{
    // Read the message
    memcpy(&currentCurrentSensorData, completeMessage.data, sizeof(CurrentSensorMessage));
}

bool VirtualCanBus::getCanMessage(tap::can::CanBus canbus, modm::can::Message* message)
{
    modm::can::Message msg;

    if (canbus == tap::can::CanBus::CAN_BUS1)
    {
        if (CAN1_queue.isEmpty())
        {
            return false;
        }
        msg = CAN1_queue.get();
    }
    else
    {
        if (CAN2_queue.isEmpty())
        {
            return false;
        }
        msg = CAN2_queue.get();
    }

    memcpy(&msg, message, sizeof(modm::can::Message));

    return true;
}

bool VirtualCanBus::sendMessage(tap::can::CanBus canbus, const modm::can::Message& message)
{
    DJISerial::SerialMessage<sizeof(modm::can::Message)> sendMessage;
    sendMessage.messageType = canbus == tap::can::CanBus::CAN_BUS1 ? MessageTypes::CANBUS1_MESSAGE
                                                                   : MessageTypes::CANBUS2_MESSAGE;
    memcpy(sendMessage.data, &message, sizeof(modm::can::Message));

    sendMessage.setCRC16();

    return drivers->uart.write(port, reinterpret_cast<uint8_t*>(&sendMessage), sizeof(sendMessage));
}

}  // namespace aruwsrc::virtualMCB