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

#include "aruwsrc/communication/can/cap-bank/capacitor_bank.hpp"

using namespace testing;

using namespace aruwsrc::communication::can::cap_bank;

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

TEST_F(CapBankTests, status_v2_is_parsed)
{
    // state=Charge(2), current=1500mA (0x05DC LE), voltage=12000mV (0x2EE0 LE),
    // energy=42%, available raw 10 -> 10 * 4 = 40 W. Mirrors the cap-bank firmware's
    // can_messages.rs `status_v2` test vector.
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::STATUS;
    message.data[1] = State::CHARGE;
    message.data[2] = 0xDC;
    message.data[3] = 0x05;
    message.data[4] = 0xE0;
    message.data[5] = 0x2E;
    message.data[6] = 42;
    message.data[7] = 10;

    capBank.processMessage(message);

    EXPECT_EQ(State::CHARGE, capBank.getState());
    EXPECT_FALSE(capBank.hasError());
    EXPECT_TRUE(capBank.isEnabled());
    EXPECT_NEAR(1.5f, capBank.getCurrent(), 1e-3);
    EXPECT_NEAR(12.0f, capBank.getVoltage(), 1e-3);
    EXPECT_EQ(42, capBank.getEnergyPercent());
    EXPECT_EQ(40, capBank.getAvailableSupplyPower());
}

TEST_F(CapBankTests, status_v2_error_flag_in_bit7)
{
    // 0x83 = error flag (0x80) | Boost (3). Mirrors the firmware's
    // `status_v2_error_flag_in_bit7` test.
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::STATUS;
    memset(message.data + 1, 0, 7);
    message.data[1] = 0x83;

    capBank.processMessage(message);

    EXPECT_EQ(State::BOOST, capBank.getState());
    EXPECT_TRUE(capBank.hasError());
}

TEST_F(CapBankTests, status_does_not_transmit)
{
    // In v2 the MCB does not reply to STATUS; processMessage only parses telemetry.
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::STATUS;
    memset(message.data + 1, 0, 7);

    EXPECT_CALL(drivers.can, sendMessage).Times(0);

    capBank.processMessage(message);
}

TEST_F(CapBankTests, send_cap_command_packs_mode_and_ref_limit_only)
{
    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData).WillRepeatedly(Return(true));
    tap::communication::serial::RefSerial::Rx::RobotData robotData;
    robotData.chassis.powerConsumptionLimit = 80;  // 80 W / 4 = 20
    EXPECT_CALL(drivers.refSerial, getRobotData).WillRepeatedly(ReturnRef(robotData));

    modm::can::Message sent;
    EXPECT_CALL(drivers.can, sendMessage).WillOnce(DoAll(SaveArg<1>(&sent), Return(true)));

    capBank.sendCapCommand(CapCommandMode::CHARGE);

    EXPECT_EQ(static_cast<uint8_t>(MessageType::CAP_COMMAND), sent.data[0]);
    EXPECT_EQ(static_cast<uint8_t>(CapCommandMode::CHARGE), sent.data[1]);
    EXPECT_EQ(0, sent.data[2]);
    EXPECT_EQ(0, sent.data[3]);
    EXPECT_EQ(0, sent.data[4]);
    EXPECT_EQ(0, sent.data[5]);
    EXPECT_EQ(20, sent.data[6]);  // ref_limit 80 W / 4
    EXPECT_EQ(0, sent.data[7]);
}

TEST_F(CapBankTests, send_cap_command_zeros_ref_limit_without_ref)
{
    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData).WillRepeatedly(Return(false));

    modm::can::Message sent;
    EXPECT_CALL(drivers.can, sendMessage).WillOnce(DoAll(SaveArg<1>(&sent), Return(true)));

    capBank.sendCapCommand(CapCommandMode::OFF);

    EXPECT_EQ(static_cast<uint8_t>(MessageType::CAP_COMMAND), sent.data[0]);
    EXPECT_EQ(static_cast<uint8_t>(CapCommandMode::OFF), sent.data[1]);
    EXPECT_EQ(0, sent.data[6]);
    EXPECT_EQ(0, sent.data[7]);
}

TEST_F(CapBankTests, capbank_goes_offline_when_heartbeat_expires)
{
    modm::can::Message message(CAP_BANK_CAN_ID, 8);
    message.setExtended(false);
    message.data[0] = MessageType::STATUS;
    memset(message.data + 1, 0, 7);

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
    memset(message.data + 1, 0, 7);

    capBank.processMessage(message);
    EXPECT_TRUE(capBank.isOnline());

    clock.time = 81;

    capBank.processMessage(message);
    EXPECT_TRUE(capBank.isOnline());
}
