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

#include "tap/architecture/clock.hpp"
#include "tap/architecture/endianness_wrappers.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/drivers.hpp"

using namespace aruwsrc::can;
using namespace aruwsrc;
using namespace testing;
using namespace tap::arch::clock;
using namespace tap::communication::sensors::imu::mpu6500;

TEST(TurretMCBCanComm, sendData_hopper_cover_data)
{
    ClockStub clock;

    Drivers drivers;
    TurretMCBCanComm dut(&drivers);

    modm::can::Message blankMsg(0x1fe, 1, {0}, false);
    modm::can::Message filledMsg(0x1fe, 1, {1}, false);

    EXPECT_CALL(drivers.can, sendMessage(tap::can::CanBus::CAN_BUS1, Eq(blankMsg)));
    EXPECT_CALL(drivers.can, sendMessage(tap::can::CanBus::CAN_BUS1, Eq(filledMsg)));

    clock.time = 10'000;
    dut.setOpenHopperCover(false);
    dut.sendData();

    clock.time = 20'000;
    dut.setOpenHopperCover(true);
    dut.sendData();
}

TEST(TurretMCBCanComm, sendData_calibrate_imu_data)
{
    ClockStub clock;

    Drivers drivers;
    TurretMCBCanComm dut(&drivers);

    modm::can::Message blankMsg(0x1fe, 1, {0}, false);
    modm::can::Message filledMsg(0x1fe, 1, {0b10}, false);

    EXPECT_CALL(drivers.can, sendMessage(tap::can::CanBus::CAN_BUS1, blankMsg));
    EXPECT_CALL(drivers.can, sendMessage(tap::can::CanBus::CAN_BUS1, filledMsg));

    clock.time = 10'000;
    dut.sendImuCalibrationRequest();
    dut.sendData();

    clock.time = 20'000;
    dut.sendData();
}

TEST(TurretMCBCanComm, sendData_laser_data)
{
    ClockStub clock;

    Drivers drivers;
    TurretMCBCanComm dut(&drivers);

    modm::can::Message blankMsg(0x1fe, 1, {0}, false);
    modm::can::Message filledMsg(0x1fe, 1, {0b100}, false);

    EXPECT_CALL(drivers.can, sendMessage(tap::can::CanBus::CAN_BUS1, Eq(blankMsg)));
    EXPECT_CALL(drivers.can, sendMessage(tap::can::CanBus::CAN_BUS1, Eq(filledMsg)));

    clock.time = 10'000;
    dut.setLaserStatus(false);
    dut.sendData();

    clock.time = 20'000;
    dut.setLaserStatus(true);
    dut.sendData();
}

TEST(TurretMCBCanComm, receive_limit_switch_info)
{
    ClockStub clock;

    Drivers drivers;
    TurretMCBCanComm dut(&drivers);

    ON_CALL(drivers.canRxHandler, attachReceiveHandler)
        .WillByDefault([&](tap::can::CanRxListener* const listener) {
            drivers.canRxHandler.CanRxHandler::attachReceiveHandler(listener);
        });

    modm::can::Message limitSwitchMsg(0x1fb, 1, {1}, false);
    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS1, _))
        .WillByDefault([&](tap::can::CanBus, modm::can::Message* message) {
            *message = limitSwitchMsg;
            return true;
        });

    dut.init();

    drivers.canRxHandler.CanRxHandler::pollCanData();

    EXPECT_TRUE(dut.getLimitSwitchDepressed());
}

TEST(TurretMCBCanComm, receive_turret_data)
{
    ClockStub clock;

    Drivers drivers;
    TurretMCBCanComm dut(&drivers);

    ON_CALL(drivers.canRxHandler, attachReceiveHandler)
        .WillByDefault([&](tap::can::CanRxListener* const listener) {
            drivers.canRxHandler.CanRxHandler::attachReceiveHandler(listener);
        });

    modm::can::Message yawMessage(0x1fd, 8, 0, false);
    modm::can::Message pitchMessage(0x1fc, 8, 0, false);
    modm::can::Message* messageToSend = &yawMessage;

    tap::arch::convertToLittleEndian<int16_t>(0x1234, yawMessage.data);
    tap::arch::convertToLittleEndian<int16_t>(0x4567, yawMessage.data + 2);
    tap::arch::convertToLittleEndian<uint8_t>(0x12, yawMessage.data + 4);
    tap::arch::convertToLittleEndian<uint16_t>(0x1234, yawMessage.data + 5);

    tap::arch::convertToLittleEndian<int16_t>(0x4321, pitchMessage.data);
    tap::arch::convertToLittleEndian<int16_t>(0x7654, pitchMessage.data + 2);
    tap::arch::convertToLittleEndian<uint8_t>(0x12, pitchMessage.data + 4);
    tap::arch::convertToLittleEndian<uint16_t>(0x5678, pitchMessage.data + 5);

    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS1, _))
        .WillByDefault([&](tap::can::CanBus, modm::can::Message* message) {
            *message = *messageToSend;
            return true;
        });

    dut.init();

    drivers.canRxHandler.CanRxHandler::pollCanData();
    messageToSend = &pitchMessage;
    drivers.canRxHandler.CanRxHandler::pollCanData();

    EXPECT_NEAR(
        modm::toRadian(360.0f / UINT16_MAX) * static_cast<int16_t>(0x1234),
        dut.getYaw(),
        1E-5);
    EXPECT_NEAR(
        modm::toRadian(static_cast<int16_t>(0x4567) / Mpu6500::LSB_D_PER_S_TO_D_PER_S),
        dut.getYawVelocity(),
        1E-5);
    EXPECT_NEAR(
        modm::toRadian(360.0f / UINT16_MAX) * static_cast<int16_t>(0x4321),
        dut.getPitch(),
        1E-5);
    EXPECT_NEAR(
        modm::toRadian(static_cast<int16_t>(0x7654) / Mpu6500::LSB_D_PER_S_TO_D_PER_S),
        dut.getPitchVelocity(),
        1E-5);
    EXPECT_EQ(0X12345678, dut.getIMUDataTimestamp());

    EXPECT_TRUE(dut.isConnected());

    clock.time = 10'000;

    EXPECT_FALSE(dut.isConnected());
}

TEST(TurretMCBCanComm, sendTimeSyncData)
{
    ClockStub clock;
    clock.time = 10'000;

    Drivers drivers;
    TurretMCBCanComm dut(&drivers);

    ON_CALL(drivers.canRxHandler, attachReceiveHandler)
        .WillByDefault([&](tap::can::CanRxListener* const listener) {
            drivers.canRxHandler.CanRxHandler::attachReceiveHandler(listener);
        });

    modm::can::Message syncReqMessage(0x1f9, 0, 0, false);

    ON_CALL(drivers.can, getMessage(tap::can::CanBus::CAN_BUS1, _))
        .WillByDefault([syncReqMessage](tap::can::CanBus, modm::can::Message* message) {
            *message = syncReqMessage;
            return true;
        });

    modm::can::Message syncMessage(0x1fa, 4, 0, false);
    tap::arch::convertToLittleEndian(getTimeMicroseconds(), syncMessage.data);
    EXPECT_CALL(drivers.can, sendMessage(_, Eq(syncMessage)));

    dut.init();

    drivers.canRxHandler.CanRxHandler::pollCanData();
}
