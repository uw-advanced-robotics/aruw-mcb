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

#include "serial_mcb_lite.hpp"

#include "tap/communication/can/can.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::virtualMCB
{
SerialMCBLite::SerialMCBLite(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port)
    : DJISerial(drivers, port),
      canRxHandler(VirtualCanRxHandler(drivers)),
      motorTxHandler(VirtualDJIMotorTxHandler(drivers)),
      currentSensor(),
      imu(),
      digital(),
      analog(),
      pwm(),
      leds(),
      port(port),
      currentIMUData(),
      currentCurrentSensorData()
{
}

void SerialMCBLite::initialize()
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

void SerialMCBLite::sendData()
{
    if (drivers->uart.isWriteFinished(port))
    {
        motorTxHandler.encodeAndSendCanData();
        // 100 bytes
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

        if (imu.requestCalibrationFlag)
        {  // 10 bytes
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(imu.calibrateIMUMessage)),
                sizeof(imu.calibrateIMUMessage));
            imu.requestCalibrationFlag = false;
        }

        if (digital.hasNewMessageData)
        {  // 27 bytes
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(digital.outputPinValuesMessage)),
                sizeof(digital.outputPinValuesMessage));
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(digital.pinModesMessage)),
                sizeof(digital.pinModesMessage));
            digital.hasNewMessageData = false;
        }

        if (pwm.hasNewMessageData)
        {  // 66 bytes
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(pwm.pinDutyMessage)),
                sizeof(pwm.pinDutyMessage));
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(pwm.timerStartedMessage)),
                sizeof(pwm.timerStartedMessage));
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(pwm.timerFrequencyMessage)),
                sizeof(pwm.timerFrequencyMessage));
            pwm.hasNewMessageData = false;
        }

        if (leds.hasNewMessageData)
        {  // 19 bytes
            drivers->uart.write(
                port,
                reinterpret_cast<uint8_t*>(&(leds.ledControlMessage)),
                sizeof(leds.ledControlMessage));
            leds.hasNewMessageData = false;
        }
    }
}

void SerialMCBLite::messageReceiveCallback(const ReceivedSerialMessage& completeMessage)
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
            case MessageTypes::CURRENT_SENSOR_MESSAGE:
                processCurrentSensorMessage(completeMessage);
                break;
            case MessageTypes::ANALOG_PIN_READ_MESSAGE:
                processAnalogMessage(completeMessage);
                break;
            case MessageTypes::DIGITAL_PID_READ_MESSAGE:
                processDigitalMessage(completeMessage);
                break;
            default:
                break;
        }
    }
}

void SerialMCBLite::processCanMessage(
    const ReceivedSerialMessage& completeMessage,
    tap::can::CanBus canbus)
{
    memcpy(
        &(canbus == tap::can::CanBus::CAN_BUS1 ? can1Data : can2Data),
        completeMessage.data,
        sizeof(can1Data));
    for (int i = 0; i < 8; i++)
    {
        modm::can::Message msg;
        for (uint8_t i = 0; i < 8; i++)
        {
            // Get back the motor num
            msg.identifier = i + tap::motor::MotorId::MOTOR1;
            memcpy(&msg.data, &completeMessage.data[i * sizeof(msg.data)], sizeof(msg.data));
            canRxHandler.refresh(canbus, msg);
        }
    }
}

void SerialMCBLite::processIMUMessage(const ReceivedSerialMessage& completeMessage)
{
    memcpy(&currentIMUData, completeMessage.data, sizeof(currentIMUData));
    imu.pitch = currentIMUData.pitch;
    imu.roll = currentIMUData.roll;
    imu.yaw = currentIMUData.yaw;
    imu.Gx = currentIMUData.Gx;
    imu.Gy = currentIMUData.Gy;
    imu.Gz = currentIMUData.Gz;
    imu.Ax = currentIMUData.Ax;
    imu.Ay = currentIMUData.Ay;
    imu.Az = currentIMUData.Az;
    imu.imuState = currentIMUData.imuState;
    imu.temperature = currentIMUData.temperature;
}

void SerialMCBLite::processCurrentSensorMessage(const ReceivedSerialMessage& completeMessage)
{
    memcpy(&currentCurrentSensorData, completeMessage.data, sizeof(currentCurrentSensorData));
    currentSensor.current = currentCurrentSensorData.current;
}

void SerialMCBLite::processAnalogMessage(const ReceivedSerialMessage& completeMessage)
{
    memcpy(&currentAnalogData, completeMessage.data, sizeof(currentAnalogData));
    analog.SPinValue = currentAnalogData.SPinValue;
    analog.TPinValue = currentAnalogData.TPinValue;
    analog.UPinValue = currentAnalogData.UPinValue;
    analog.VPinValue = currentAnalogData.VPinValue;
    analog.OLEDPinValue = currentAnalogData.OLEDPinValue;
}

void SerialMCBLite::processDigitalMessage(const ReceivedSerialMessage& completeMessage)
{
    memcpy(&currentDigitalData, completeMessage.data, sizeof(currentDigitalData));
    digital.inputPinB = currentDigitalData.BPinValue;
    digital.inputPinC = currentDigitalData.CPinValue;
    digital.inputPinD = currentDigitalData.DPinValue;
    digital.inputPinButton = currentDigitalData.ButtonPinValue;
}

}  // namespace aruwsrc::virtualMCB
