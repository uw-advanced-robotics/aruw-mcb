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

#ifndef IMU_VEL_LISTENER_HPP_
#define IMU_VEL_LISTENER_HPP_

#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "virtual_can_rx_listener.hpp"

namespace aruwsrc::virtualMCB
{
class IMUVelListener : public VirtualCANRxListener
{
public:
    // These values should be hardcoded equivalents
    IMUVelListener(tap::Drivers* drivers, tap::can::CanBus canbus, VirtualCANRxHandler* canHandler)
        : VirtualCANRxListener(drivers, IMU_VEL_MESSAGE, canbus, canHandler){};

    inline void processMessage(const modm::can::Message& message) override
    {
        if (message.getIdentifier() != IMU_VEL_MESSAGE)
        {
            return;
        }

        pitchVel = message.data[0] << 8 & message.data[1];
        yawVel = message.data[2] << 8 & message.data[3];
        rollVel = message.data[4] << 8 & message.data[5];
    }

    uint16_t pitchVel, yawVel, rollVel;
};

}  // namespace aruwsrc::virtualMCB

#endif