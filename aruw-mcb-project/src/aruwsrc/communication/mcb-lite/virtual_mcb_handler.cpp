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

#include "virtual_mcb_handler.hpp"

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::virtualMCB
{
VirtualMCBHandler::VirtualMCBHandler(
    tap::Drivers* drivers,
    tap::communication::serial::Uart::UartPort port)
    : DJISerial(drivers, port),
      canRxHandler(VirtualCanRxHandler(drivers)),
      motorTxHandler(VirtualDJIMotorTxHandler(drivers)),
      port(port),
      currentIMUData(),
      currentCurrentSensorData(),
      calibrateIMUMessage()

{
}

void VirtualMCBHandler::initialize()
{
    switch (this->port)
    {
        case Uart::UartPort::Uart1:
            drivers->uart.init<Uart::UartPort::Uart1, UART_BAUDRATE>();
            break;
        case Uart::UartPort::Uart2:
            drivers->uart.init<Uart::UartPort::Uart2, UART_BAUDRATE>();
            break;
        case Uart::UartPort::Uart3:
            drivers->uart.init<Uart::UartPort::Uart3, UART_BAUDRATE>();
            break;
        case Uart::UartPort::Uart6:
            drivers->uart.init<Uart::UartPort::Uart6, UART_BAUDRATE>();
            break;
        case Uart::UartPort::Uart7:
            drivers->uart.init<Uart::UartPort::Uart7, UART_BAUDRATE>();
            break;
        case Uart::UartPort::Uart8:
            drivers->uart.init<Uart::UartPort::Uart8, UART_BAUDRATE>();
            break;
        default:
            break;
    }
}

void VirtualMCBHandler::sendData()
{
    if (drivers->uart.isWriteFinished(port))
    {
        motorTxHandler.encodeAndSendCanData();
        drivers->uart.write(
            port,
            reinterpret_cast<uint8_t*>(&(motorTxHandler.can1MessageLowSend)),
            sizeof(motorTxHandler.can1MessageLowSend));
        drivers->uart.write(
            port,
            reinterpret_cast<uint8_t*>(&(motorTxHandler.can1MessageHighSend)),
            sizeof(motorTxHandler.can1MessageHighSend));
        drivers->uart.write(
            port,
            reinterpret_cast<uint8_t*>(&(motorTxHandler.can2MessageLowSend)),
            sizeof(motorTxHandler.can2MessageLowSend));
        drivers->uart.write(
            port,
            reinterpret_cast<uint8_t*>(&(motorTxHandler.can2MessageHighSend)),
            sizeof(motorTxHandler.can2MessageHighSend));

        if (sendIMUCalibrationMessage)
        {
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(calibrateIMUMessage)),
                sizeof(calibrateIMUMessage));
            sendIMUCalibrationMessage = false;
        }
    }
}

void VirtualMCBHandler::calibrateIMU()
{
    calibrateIMUMessage.messageType = MessageTypes::CALIBRATE_IMU;
    calibrateIMUMessage.setCRC16();
    sendIMUCalibrationMessage = true;
}

IMUMessage& VirtualMCBHandler::getIMUMessage() { return currentIMUData; }

CurrentSensorMessage& VirtualMCBHandler::getCurrentSensorMessage()
{
    return currentCurrentSensorData;
}

void VirtualMCBHandler::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
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
                processCurrentSensorMessage(completeMessage);
                break;
            default:
                break;
        }
    }
}

void VirtualMCBHandler::processCanMessage(
    const ReceivedSerialMessage& completeMessage,
    tap::can::CanBus canbus)
{
    modm::can::Message msg;
    memcpy(&msg, completeMessage.data, sizeof(modm::can::Message));
    canRxHandler.refresh(canbus, msg);
}

void VirtualMCBHandler::processIMUMessage(const ReceivedSerialMessage& completeMessage)
{
    memcpy(&currentIMUData, completeMessage.data, sizeof(IMUMessage));
}

void VirtualMCBHandler::processCurrentSensorMessage(const ReceivedSerialMessage& completeMessage)
{
    memcpy(&currentCurrentSensorData, completeMessage.data, sizeof(CurrentSensorMessage));
}

}  // namespace aruwsrc::virtualMCB