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

#ifndef IMU_POS_LISTENER_HPP_
#define IMU_POS_LISTENER_HPP_

#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "virtual_can_rx_listener.hpp"

namespace aruwsrc::virtualMCB
{
class IMUPosListener : public VirtualCANRxListener
{
public:
    // These values should be hardcoded equivalents
    IMUPosListener(tap::Drivers* drivers, tap::can::CanBus canbus, VirtualCANRxHandler* canHandler)
        : VirtualCANRxListener(drivers, IMU_POS_MESSAGE, canbus, canHandler){};

    inline void processMessage(const modm::can::Message& message) override
    {
        if (message.getIdentifier() != IMU_POS_MESSAGE)
        {
            return;
        }

        pitch = getFloatFromMessage(message, 0);
        yaw = getFloatFromMessage(message, 2);
        roll = getFloatFromMessage(message, 4);

        uint8_t state = message.data[6] << 8 | message.data[7];

        imuState = static_cast<tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState>(state);
    }

    float yaw, pitch, roll;
    tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState imuState =
        tap::communication::sensors::imu::mpu6500::Mpu6500::ImuState::IMU_NOT_CONNECTED;

private:
    static float getFloatFromMessage(const modm::can::Message& message, uint8_t startIndex)
    {
        uint16_t data = message.data[startIndex] << 8 | message.data[startIndex + 1];
        return (float) data / 100.0f;
    };
};  // class IMUPosListener

}  // namespace aruwsrc::virtualMCB

#endif