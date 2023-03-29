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

#ifndef IMU_LISTENER_HPP_
#define IMU_LISTENER_HPP_

#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "virtual_can_rx_listener.hpp"

namespace aruwsrc::virtualMCB
{
class IMUListener : public VirtualCANRxListener
{
    // These values should be hardcoded equivalents
    IMUListener(
        tap::Drivers* drivers,
        uint32_t id,
        tap::can::CanBus canbus,
        VirtualCANRxHandler* canHandler)
        : VirtualCANRxListener(drivers, id, canbus, canHandler){};

    inline void processMessage(const modm::can::Message& message) override {}
};

}  // namespace aruwsrc::virtualMCB

#endif