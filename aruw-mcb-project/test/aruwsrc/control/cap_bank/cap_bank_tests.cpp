/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/drivers.hpp"

#include "aruwsrc/communication/can/capacitor_bank.hpp"

using namespace testing;

using namespace aruwsrc::can::capbank;

class CapBankTests : public Test
{
public:
    CapBankTests() : drivers(), capBank(&drivers, tap::can::CanBus::CAN_BUS1, 1.0), clock() {}

    tap::Drivers drivers;
    CapacitorBank capBank;
    tap::arch::clock::ClockStub clock;
};

TEST_F(CapBankTests, initalize_connects_to_can)
{
    EXPECT_CALL(drivers.canRxHandler, attachReceiveHandler(&capBank));

    capBank.initialize();
}

TEST_F(CapBankTests, start_sends_message)
{
    EXPECT_CALL(drivers.can, sendMessage).Times(1);

    capBank.start();
}

TEST_F(CapBankTests, stop_sends_message)
{
    EXPECT_CALL(drivers.can, sendMessage).Times(1);

    capBank.stop();
}

TEST_F(CapBankTests, ping_sends_message)
{
    EXPECT_CALL(drivers.can, sendMessage).Times(1);

    capBank.ping();
}

TEST_F(CapBankTests, status_is_received)
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::STATUS;
    message.data[1] = State::RESET;
    memset(message.data + 2, 0, 6);

    EXPECT_NE(capBank.getState(), State::RESET);

    capBank.processMessage(message);

    EXPECT_EQ(capBank.getState(), State::RESET);
}

TEST_F(CapBankTests, when_receiving_status_do_not_update_power_when_no_ref)
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::STATUS;
    message.data[1] = State::RESET;
    memset(message.data + 2, 0, 6);

    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData).WillRepeatedly(Return(false));

    tap::communication::serial::RefSerial::Rx::RobotData robotData;
    EXPECT_CALL(drivers.refSerial, getRobotData).Times(0);

    robotData.chassis.powerConsumptionLimit = 100;

    EXPECT_CALL(drivers.can, sendMessage).Times(0);

    capBank.processMessage(message);
}

TEST_F(CapBankTests, when_receiving_status_update_power_when_ref)
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::STATUS;
    message.data[1] = State::RESET;
    memset(message.data + 2, 0, 6);

    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData).WillRepeatedly(Return(true));

    tap::communication::serial::RefSerial::Rx::RobotData robotData;
    EXPECT_CALL(drivers.refSerial, getRobotData).WillRepeatedly(ReturnRef(robotData));

    robotData.chassis.powerConsumptionLimit = 100;

    EXPECT_CALL(drivers.can, sendMessage);

    capBank.processMessage(message);
}

TEST_F(CapBankTests, capbank_goes_offline_when_heartbeat_expires)
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::STATUS;
    message.data[1] = State::RESET;
    memset(message.data + 2, 0, 6);

    capBank.processMessage(message);
    EXPECT_TRUE(capBank.isOnline());

    clock.time = 81;

    EXPECT_FALSE(capBank.isOnline());
}

TEST_F(CapBankTests, heartbeat_is_reset_when_receiving_status)
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::STATUS;
    message.data[1] = State::RESET;
    memset(message.data + 2, 0, 6);

    capBank.processMessage(message);
    EXPECT_TRUE(capBank.isOnline());

    clock.time = 81;

    capBank.processMessage(message);
    EXPECT_TRUE(capBank.isOnline());
}
