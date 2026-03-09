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

#include "mcb_lite.hpp"

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::communication::mcb_lite
{
MCBLite::MCBLite(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port)
    : DJISerial(drivers, port),
      canRxHandler(motor::VirtualCanRxHandler(drivers)),
      motorTxHandler(motor::VirtualDJIMotorTxHandler(drivers)),
      imu(),
      analog(),
      digital(),
      leds(),
      pwm(),
      port(port),
      currentIMUData()
{
}

void MCBLite::initialize()
{
    initialized = true;
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

void MCBLite::sendData()
{
    if (drivers->uart.isWriteFinished(port))
    {
        motorTxHandler.encodeAndSendCanData();
        // 100 bytes of CAN data
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

        if (imu.sendIMUCalibrationMessage)
        {
            // 10 bytes of IMU
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(imu.calibrateIMUMessage)),
                sizeof(imu.calibrateIMUMessage));
            imu.sendIMUCalibrationMessage = false;
        }

        if (digital.hasNewData)
        {
            // 27 bytes of digital
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(digital.outputPinMessage)),
                sizeof(digital.outputPinMessage));
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(digital.pinModeMessage)),
                sizeof(digital.pinModeMessage));
            digital.hasNewData = false;
        }

        if (leds.hasNewData)
        {
            // 19 bytes of LED
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(leds.ledStateMessage)),
                sizeof(leds.ledStateMessage));
            leds.hasNewData = false;
        }

        if (pwm.hasNewData)
        {
            // 66 bytes of PWM
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(pwm.pinDutyMessage)),
                sizeof(pwm.pinDutyMessage));
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(pwm.pwmTimerFrequencyMessage)),
                sizeof(pwm.pwmTimerFrequencyMessage));
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(pwm.pwmTimerStartMessage)),
                sizeof(pwm.pwmTimerStartMessage));
            pwm.hasNewData = false;
        }
    }
}

void MCBLite::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
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
                memcpy(&currentIMUData, completeMessage.data, sizeof(currentIMUData));
                imu.processIMUMessage(completeMessage);
                break;
            case MessageTypes::ANALOG_PIN_READ_MESSAGE:
                memcpy(&analogData, completeMessage.data, sizeof(analogData));
                analog.processAnalogMessage(completeMessage);
                break;
            case MessageTypes::DIGITAL_PID_READ_MESSAGE:
                memcpy(&digitalData, completeMessage.data, sizeof(digitalData));
                digital.processDigitalMessage(completeMessage);
                break;
            case MessageTypes::CAN1_ENCODER_MESSAGE:
                processCanEncoderMessage(completeMessage, can1Encoders);
                break;
            case MessageTypes::CAN2_ENCODER_MESSAGE:
                processCanEncoderMessage(completeMessage, can2Encoders);
                break;
            case MessageTypes::VOLTAGE_CURRENT_MESSAGE:
                processVoltageCurrentMessage(completeMessage);
                break;
            default:
                break;
        }
    }
}

void MCBLite::processCanMessage(
    const ReceivedSerialMessage& completeMessage,
    tap::can::CanBus canbus)
{
    memcpy(
        &(canbus == tap::can::CanBus::CAN_BUS1 ? can1Data : can2Data),
        completeMessage.data,
        sizeof(can1Data));
    uint8_t bitmap = completeMessage.data[sizeof(can1Data)];

    modm::can::Message msg;
    for (uint8_t i = 0; i < 8; i++)
    {
        if ((bitmap & (1 << i)) == 0)
        {
            continue;
        }

        // Get back the motor num
        msg.identifier = i + tap::motor::MotorId::MOTOR1;
        memcpy(&msg.data, &completeMessage.data[i * sizeof(msg.data)], sizeof(msg.data));
        canRxHandler.refresh(canbus, msg);
    }
}

void MCBLite::processCanEncoderMessage(
    const ReceivedSerialMessage& completeMessage,
    VirtualCanEncoder** encoders)
{
    modm::can::Message message{};

    uint8_t online = completeMessage.data[0];
    for (uint8_t i = 0; i < 8; i++)
    {
        if ((online & (1 << i)) != 0 && encoders[i] != nullptr)
        {
            memcpy(message.data, completeMessage.data + 1 + i * 4, 4);
            encoders[i]->processMessage(message);
        }
    }
}

void MCBLite::processVoltageCurrentMessage(const ReceivedSerialMessage& completeMessage)
{
    const VoltageCurrentMessage* message =
        reinterpret_cast<const VoltageCurrentMessage*>(completeMessage.data);

    if (this->voltageCurrentSensor != nullptr)
    {
        this->voltageCurrentSensor->voltage = message->voltage;
        this->voltageCurrentSensor->current = message->current;
    }
}

void MCBLite::processAnalogSensorMessage(const ReceivedSerialMessage& completeMessage)
{
    const AnalogSensorMessage* message =
        reinterpret_cast<const AnalogSensorMessage*>(completeMessage.data);
    if (this->analogSensor != nullptr)
    {
        this->analogSensor->processAnalogSensorUARTMessage(message->ai0, message->ai1);
    }
}
}  // namespace aruwsrc::communication::mcb_lite
