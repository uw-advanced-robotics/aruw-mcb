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
#include "tap/communication/sensors/current/current_sensor_interface.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "modm/container/queue.hpp"
#include "motor/virtual_can_rx_handler.hpp"
#include "motor/virtual_dji_motor_tx_handler.hpp"

using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc::virtualMCB
{
enum MessageTypes : uint8_t
{
    CANBUS1_MESSAGE = 0,
    CANBUS2_MESSAGE = 1,
    IMU_MESSAGE = 2,
    CURRENT_SENSOR_MESSAGE = 3,
    CALIBRATE_IMU = 4
};

struct IMUMessage
{
    float pitch, roll, yaw;
    float Gx, Gy, Gz;
    float Ax, Ay, Az;
    Mpu6500::ImuState imuState;
    float temperature;
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

    IMUMessage currentIMUData;

    CurrentSensorMessage currentCurrentSensorData;

private:
    void processCanMessage(const ReceivedSerialMessage& completeMessage, tap::can::CanBus canbus);

    void processIMUMessage(const ReceivedSerialMessage& completeMessage);

    void processCurrentSensorMessage(const ReceivedSerialMessage& completeMessage);

    tap::communication::serial::Uart::UartPort port;

    tap::communication::serial::DJISerial::DJISerial::SerialMessage<0> calibrateIMUMessage;
    bool sendIMUCalibrationMessage = false;
};

class VirtualCurrentSensor : public tap::communication::sensors::current::CurrentSensorInterface
{
    VirtualCurrentSensor(VirtualMCBHandler* handler) : handler(handler) {}
    void update() override {}
    float getCurrentMa() const override { return handler->currentCurrentSensorData.current; }
    aruwsrc::virtualMCB::VirtualMCBHandler* handler;
};

class VirtualIMUInterface : public tap::communication::sensors::imu::ImuInterface
{
    VirtualIMUInterface(VirtualMCBHandler* handler) : handler(handler) {}

    float getPitch() override { return handler->currentIMUData.pitch; }
    float getRoll() override { return handler->currentIMUData.roll; }
    float getYaw() override { return handler->currentIMUData.yaw; }
    float getGx() override { return handler->currentIMUData.Gx; }
    float getGy() override { return handler->currentIMUData.Gy; }
    float getGz() override { return handler->currentIMUData.Gz; }
    float getAx() override { return handler->currentIMUData.Ax; }
    float getAy() override { return handler->currentIMUData.Ay; }
    float getAz() override { return handler->currentIMUData.Az; }
    float getTemp() override { return handler->currentIMUData.temperature; }
    Mpu6500::ImuState getImuState() { return handler->currentIMUData.imuState; }

    aruwsrc::virtualMCB::VirtualMCBHandler* handler;
};

}  // namespace aruwsrc::virtualMCB

#endif