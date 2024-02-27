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

#ifndef SERIAL_MCB_LITE_HPP_
#define SERIAL_MCB_LITE_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "modm/container/queue.hpp"
#include "motor/virtual_can_rx_handler.hpp"
#include "motor/virtual_dji_motor_tx_handler.hpp"

#include "message_types.hpp"
#include "virtual_analog.hpp"
#include "virtual_current_sensor.hpp"
#include "virtual_digital.hpp"
#include "virtual_imu_interface.hpp"
#include "virtual_leds.hpp"

using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc::virtualMCB
{
/**
 * This class is used to communicate with the the virtual MCB using the UART port.
 * This class handles the sending and receiving of motor data, as well as receiving
 * IMU and current sensor data. To use: call initialize() once to setup the port baudrate.
 * Afterwards, call updateSerial() as fast as possible to process receiving data as fast as
 * possible. Call sendData per loop to send motor desired outputs at a consistent rate.
 */
class MCBLite : public tap::communication::serial::DJISerial
{
public:
    MCBLite(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port);

    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    void sendData();

    void initialize();

    constexpr static int UART_BAUDRATE = 500'000;

    VirtualCanRxHandler canRxHandler;
    VirtualDJIMotorTxHandler motorTxHandler;
    VirtualCurrentSensor currentSensor;
    VirtualIMUInterface imu;
    VirtualAnalog analog;
    VirtualDigital digital;
    VirtualLEDs leds;

private:
    void processCanMessage(const ReceivedSerialMessage& completeMessage, tap::can::CanBus canbus);

    void processCurrentSensorMessage(const ReceivedSerialMessage& completeMessage);

    tap::communication::serial::Uart::UartPort port;

    IMUMessage currentIMUData;
    uint8_t can1Data[64];
    uint8_t can2Data[64];
    AnalogInputPinMessage analogData;
    DigitalInputPinMessage digitalData;
};
}  // namespace aruwsrc::virtualMCB

#endif
