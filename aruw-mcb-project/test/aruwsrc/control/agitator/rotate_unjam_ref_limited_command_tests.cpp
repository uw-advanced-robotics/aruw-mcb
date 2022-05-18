/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <memory>

#include <gtest/gtest.h>

#include "tap/mock/integrable_setpoint_subsystem_mock.hpp"
#include "tap/mock/move_integral_command_mock.hpp"
#include "tap/mock/unjam_integral_command_mock.hpp"

#include "aruwsrc/control/agitator/rotate_unjam_ref_limited_command.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::control::setpoint;
using namespace aruwsrc::agitator;
using namespace aruwsrc::mock;
using namespace testing;
using namespace tap::communication::serial;

class RotateUnjamRefLimitedCommandTest
    : public TestWithParam<std::tuple<RefSerialData::Rx::MechanismID, uint16_t> >
{
protected:
    RotateUnjamRefLimitedCommandTest() : agitator(&drivers), moveCmd(agitator), unjamCmd(agitator)
    {
    }

    void SetUp() override
    {
        ON_CALL(agitator, isJammed).WillByDefault(Return(false));

        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
        ON_CALL(drivers.refSerial, getRefSerialReceivingData)
            .WillByDefault(ReturnPointee(&receivingRefSerial));

        ON_CALL(moveCmd, isReady).WillByDefault(Return(true));
        ON_CALL(moveCmd, isFinished).WillByDefault(Return(false));
        ON_CALL(moveCmd, getRequirementsBitwise)
            .WillByDefault(Return(1UL << agitator.getGlobalIdentifier()));

        ON_CALL(unjamCmd, isReady).WillByDefault(Return(true));
        ON_CALL(unjamCmd, isFinished).WillByDefault(Return(false));
        ON_CALL(unjamCmd, getRequirementsBitwise)
            .WillByDefault(Return(1UL << agitator.getGlobalIdentifier()));

        cmd = std::unique_ptr<RotateUnjamRefLimitedCommand>(new RotateUnjamRefLimitedCommand(
            drivers,
            agitator,
            moveCmd,
            unjamCmd,
            std::get<0>(GetParam()),
            std::get<1>(GetParam())));
    }

    void setHeatAndHeatLimit(uint16_t heat, uint16_t heatLimit)
    {
        switch (std::get<0>(GetParam()))
        {
            case RefSerialData::Rx::MechanismID::TURRET_17MM_1:
                robotData.turret.heat17ID1 = heat;
                robotData.turret.heatLimit17ID1 = heatLimit;
                break;
            case RefSerialData::Rx::MechanismID::TURRET_17MM_2:
                robotData.turret.heat17ID2 = heat;
                robotData.turret.heatLimit17ID2 = heatLimit;
                break;
            case RefSerialData::Rx::MechanismID::TURRET_42MM:
                robotData.turret.heat42 = heat;
                robotData.turret.heatLimit42 = heatLimit;
                break;
            default:
                assert(true);
        }
    }

    RefSerialData::Rx::RobotData robotData = {};
    bool receivingRefSerial = false;
    aruwsrc::Drivers drivers;
    NiceMock<tap::mock::IntegrableSetpointSubsystemMock> agitator;
    NiceMock<tap::mock::MoveIntegralCommandMock> moveCmd;
    NiceMock<tap::mock::UnjamIntegralCommandMock> unjamCmd;
    std::unique_ptr<RotateUnjamRefLimitedCommand> cmd;
};

TEST_P(RotateUnjamRefLimitedCommandTest, command_is_ready_if_not_receiving_ref_serial)
{
    receivingRefSerial = false;

    EXPECT_TRUE(cmd->isReady());
}

// Command should be ready if it's just >= heat_buffer away from heat limit
TEST_P(RotateUnjamRefLimitedCommandTest, heat_limited_command_is_ready_within_limit)
{
    receivingRefSerial = true;

    uint16_t heat = 10;
    uint16_t heatLimit = std::get<1>(GetParam()) + heat;

    setHeatAndHeatLimit(heat, heatLimit);

    EXPECT_TRUE(cmd->isReady());
}

// Command should NOT be ready if heat > heat limit - heat buffer
TEST_P(RotateUnjamRefLimitedCommandTest, heat_limited_command_isnt_ready_above_limit)
{
    receivingRefSerial = true;

    uint16_t heat = 10;
    uint16_t heatLimit = 1 + std::get<1>(GetParam());

    setHeatAndHeatLimit(heat, heatLimit);

    EXPECT_FALSE(cmd->isReady());
}

// Command should NOT be ready if heat much > than heat buffer
TEST_P(RotateUnjamRefLimitedCommandTest, heat_limited_command_isnt_ready_far)
{
    receivingRefSerial = true;

    uint16_t heat = 1 + std::get<1>(GetParam()) + 100;
    uint16_t heatLimit = 1 + std::get<1>(GetParam());

    setHeatAndHeatLimit(heat, heatLimit);

    EXPECT_FALSE(cmd->isReady());
}

TEST_P(RotateUnjamRefLimitedCommandTest, heat_limited_command_ready_invalid_heat_heatLimit)
{
    receivingRefSerial = true;

    uint16_t heat = 100;
    uint16_t heatLimit = 0;

    setHeatAndHeatLimit(heat, heatLimit);

    EXPECT_TRUE(cmd->isReady());

    heat = 0xffff;
    heatLimit = 10;

    setHeatAndHeatLimit(heat, heatLimit);

    EXPECT_TRUE(cmd->isReady());

    heat = 0xffff;
    heatLimit = 0xffff;

    setHeatAndHeatLimit(heat, heatLimit);

    EXPECT_TRUE(cmd->isReady());
}

std::vector<RefSerialData::Rx::MechanismID> turretMechList = {
    RefSerialData::Rx::MechanismID::TURRET_17MM_1,
    RefSerialData::Rx::MechanismID::TURRET_17MM_2,
    RefSerialData::Rx::MechanismID::TURRET_42MM,
};

std::vector<uint16_t> heatLimitBuffersToTest = {
    0,
    10,
    100,
};

INSTANTIATE_TEST_SUITE_P(
    RotateUnjamRefLimitedCommand,
    RotateUnjamRefLimitedCommandTest,
    Combine(ValuesIn(turretMechList), ValuesIn(heatLimitBuffersToTest)));
