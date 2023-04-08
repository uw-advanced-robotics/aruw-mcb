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

#include <gtest/gtest.h>

#include "tap/communication/can/can_bus.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/mcb-lite/can/virtual_can_bus.hpp"
#include "aruwsrc/communication/mcb-lite/listeners/imu_pos_listener.hpp"
#include "aruwsrc/communication/mcb-lite/listeners/listener_constants.hpp"
#include "aruwsrc/communication/mcb-lite/listeners/virtual_can_rx_listener.hpp"

using namespace aruwsrc::virtualMCB;
using namespace testing;

class IMUPosListenerTest : public Test
{
public:
    IMUPosListenerTest()
        : drivers(),
          virtualCanBus(),
          canHandler(&drivers, virtualCanBus),
          listener(&drivers, canbus, &canHandler)
    {
        canbus = tap::can::CanBus::CAN_BUS1;
        virtualCanBus =
            new VirtualCanBus<tap::communication::serial::Uart::UartPort::Uart1>(&drivers);
    }

    tap::Drivers drivers;
    tap::can::CanBus canbus;
    VirtualCanBus<tap::communication::serial::Uart::UartPort::Uart1>* virtualCanBus;
    VirtualCANRxHandler canHandler;
    IMUPosListener listener;
};

TEST_F(IMUPosListenerTest, test_zero_message_hamburger)
{
    modm::can::Message message(IMU_POS_MESSAGE, 8);

    listener.processMessage(message);

    EXPECT_EQ(listener.pitch, 0);
    EXPECT_EQ(listener.yaw, 0x0203);
    EXPECT_EQ(listener.roll, 0x0405);
}