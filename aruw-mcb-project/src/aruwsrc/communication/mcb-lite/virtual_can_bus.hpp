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

#ifndef VIRTUAL_CAN_BUS_HPP_
#define VIRTUAL_CAN_BUS_HPP_

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/can/can_rx_handler.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "modm/container/queue.hpp"

namespace aruwsrc::virtualMCB
{
enum MessageRecieveTypes : uint8_t
{
    CANBUS1_MESSAGE = 0,
    CANBUS2_MESSAGE = 1,
    IMU_MESSAGE = 2,
    GPIO_MESSAGE = 3
} modm_packed;

enum MessageSendTypes : u_int16_t
{
    CAN_BUS1 = 129,
    CAN_BUS2 = 130
} modm_packed;

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

class VirtualCanBus : public tap::communication::serial::DJISerial
{
public:
    VirtualCanBus(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port);

    ~VirtualCanBus(){};

    bool getCanMessage(tap::can::CanBus canbus, modm::can::Message* message);

    bool getIMUMessage();

    bool getGPIOMessage();

    bool sendMessage(tap::can::CanBus canbus, const modm::can::Message& message);

    void messageReceiveCallback(const ReceivedSerialMessage& completeMessage) override;

private:
    void processCanMessage(const ReceivedSerialMessage& completeMessage, tap::can::CanBus canbus);

    void processIMUMessage(const ReceivedSerialMessage& completeMessage);

    IMUMessage currentIMUData;

    tap::Drivers* drivers;

    tap::communication::serial::Uart::UartPort thePort;

    // Blame eli if this does bad hardware kilobytes things
    modm::BoundedQueue<modm::can::Message, 254> CAN1_queue;
    modm::BoundedQueue<modm::can::Message, 254> CAN2_queue;

    // choosing this cuz in binary this would be 10000001 so we don't read empty buffer as can bus 1
    static constexpr uint16_t CANBUS_ID_OFFSET = 129;
};

}  // namespace aruwsrc::virtualMCB

#endif