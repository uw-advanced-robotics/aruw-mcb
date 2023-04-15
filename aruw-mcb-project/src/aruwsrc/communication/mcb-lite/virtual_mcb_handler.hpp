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

#ifndef VIRTUAL_MCB_HANDLER_HPP_
#define VIRTUAL_MCB_HANDLER_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "modm/container/queue.hpp"
#include "motor/virtual_can_rx_handler.hpp"
#include "motor/virtual_dji_motor_tx_handler.hpp"

namespace aruwsrc::virtualMCB
{
enum MessageTypes : uint8_t
{
    CANBUS1_MESSAGE = 0,
    CANBUS2_MESSAGE = 1,
    IMU_MESSAGE = 2,
    GPIO_MESSAGE = 3,
    CALIBRATE_IMU = 4
};

struct IMUMessage
{
    float pitch, roll, yaw;
    float pitchRate, rollRate, yawRate;
    float xAccel, yAccel, zAccel;
    tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState imuState;
} modm_packed;

struct CurrentSensorMessage
{
    float current;
} modm_packed;

class VirtualMCBHandler : public tap::communication::serial::DJISerial
{
    /**
     * This class is used to communicate with the the virtual MCB using the UART port.
     * This class handles the sending and receiving of motor data, as well as receiving
     * IMU and current sensor data. Call refresh() to update the data.
     */
public:
    VirtualMCBHandler(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port);

    bool getCanMessage(tap::can::CanBus canbus, modm::can::Message* message);

    IMUMessage& getIMUMessage();

    CurrentSensorMessage& getCurrentSensorMessage();

    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

    void sendData();

    void calibrateIMU();

    void initialize();

    constexpr static int UART_BAUDRATE = 1'000'000;

    VirtualCanRxHandler canRxHandler;
    VirtualDJIMotorTxHandler motorTxHandler;

private:
    void processCanMessage(const ReceivedSerialMessage& completeMessage, tap::can::CanBus canbus);

    void processIMUMessage(const ReceivedSerialMessage& completeMessage);

    void processCurrentSensorMessage(const ReceivedSerialMessage& completeMessage);

    tap::communication::serial::Uart::UartPort port;

    IMUMessage currentIMUData;

    CurrentSensorMessage currentCurrentSensorData;

    tap::communication::serial::DJISerial::DJISerial::SerialMessage<0> calibrateIMUMessage;
    bool sendIMUCalibrationMessage = false;
};

}  // namespace aruwsrc::virtualMCB

#endif