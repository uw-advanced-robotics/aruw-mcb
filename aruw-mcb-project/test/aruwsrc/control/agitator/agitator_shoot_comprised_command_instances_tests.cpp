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

/**
 * The only tests performed here are to validate heat limiting is performed
 * properly.
 */

#include <gtest/gtest.h>

#include "aruwsrc/control/agitator/agitator_shoot_comprised_command_instances.hpp"
#include "aruwsrc/mock/agitator_subsystem_mock.hpp"

using namespace tap::control::setpoint;
using namespace aruwsrc::agitator;
using namespace aruwsrc::mock;
using namespace testing;
using namespace tap::serial;

#define SETUP_TEST(ComprisedCommand, ...)               \
    aruwsrc::Drivers drivers;                           \
    NiceMock<AgitatorSubsystemMock> agitator(&drivers); \
    ComprisedCommand shootCommand(&drivers, &agitator, 1, 1, 1, ##__VA_ARGS__);

#define SETUP_IS_READY_EXPECTATIONS(receivingRefSerial)                          \
    EXPECT_CALL(drivers.refSerial, getRobotData).WillOnce(ReturnRef(robotData)); \
    EXPECT_CALL(drivers.refSerial, getRefSerialReceivingData).WillOnce(Return(receivingRefSerial));

// Validation for heat limiting confirmed by whether or not the command reports it is finished
// after initialize is called

TEST(MoveUnjamRefLimitedCommand, command_is_ready_if_heat_limiting_disabled)
{
    // Create shoot command with heat limiting off
    SETUP_TEST(MoveUnjamRefLimitedCommand, false, 0);
    RefSerial::Rx::RobotData robotData;
    robotData.turret.heat17ID1 = 10;
    // should work when receiving ref serial
    SETUP_IS_READY_EXPECTATIONS(true);
    EXPECT_EQ(shootCommand.isReady(), true);
    // and when not receiving ref serial
    SETUP_IS_READY_EXPECTATIONS(false);
    EXPECT_EQ(shootCommand.isReady(), true);
}

TEST(MoveUnjamRefLimitedCommand, command_is_ready_if_not_receiving_ref_serial)
{
    SETUP_TEST(MoveUnjamRefLimitedCommand, true, 0);
    RefSerial::Rx::RobotData robotData;
    // Oh no! we aren't receiving ref serial data
    SETUP_IS_READY_EXPECTATIONS(false);
    // Shoot anyways (this is currently expected behavior ask Matthew why)
    EXPECT_EQ(shootCommand.isReady(), true);
}

// Command should be ready if it's just >= heat_buffer away from heat limit
TEST(MoveUnjamRefLimitedCommand, heat_limited_command_is_ready_within_limit)
{
    SETUP_TEST(MoveUnjamRefLimitedCommand, true, 10);
    RefSerial::Rx::RobotData robotData;
    robotData.turret.heatLimit17ID1 = 200;
    robotData.turret.heat17ID1 = 200 - 10;
    SETUP_IS_READY_EXPECTATIONS(true);
    EXPECT_EQ(shootCommand.isReady(), true);
}

// Command should NOT be ready if it's heat limited and heat > heat limit - heat buffer
TEST(MoveUnjamRefLimitedCommand, heat_limited_command_isnt_ready_above_limit)
{
    SETUP_TEST(MoveUnjamRefLimitedCommand, true, 10);
    RefSerial::Rx::RobotData robotData;
    robotData.turret.heatLimit17ID1 = 200;
    robotData.turret.heat17ID1 = 200 - 10 + 1;

    SETUP_IS_READY_EXPECTATIONS(true);
    EXPECT_EQ(shootCommand.isReady(), false);
}

// Command should NOT be ready if subsystem is offline
TEST(MoveUnjamRefLimitedCommand, heat_limited_command_isnt_ready_when_subsystem_offline)
{
    SETUP_TEST(MoveUnjamRefLimitedCommand, true, 10);
    RefSerial::Rx::RobotData robotData;
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    EXPECT_CALL(agitator, isOnline).WillOnce(Return(false));
    EXPECT_EQ(shootCommand.isReady(), false);
}
