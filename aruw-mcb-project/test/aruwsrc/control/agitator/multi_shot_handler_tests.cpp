/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/mock/hold_repeat_command_mapping_mock.hpp"

#include "aruwsrc/control/agitator/multi_shot_handler.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/fire_rate_manager_mock.hpp"

using namespace testing;
using namespace aruwsrc::control::agitator;

class MultiShotHandlerTest : public Test
{
protected:
    MultiShotHandlerTest()
        : cmdMapping(
              &drivers,
              std::vector<tap::control::Command *>(),
              tap::control::RemoteMapState(),
              false),
          multiShotHandler(cmdMapping, fireRateManager, 3)
    {
    }

    void SetUp() override {}

    tap::Drivers drivers;
    NiceMock<tap::mock::HoldRepeatCommandMappingMock> cmdMapping;
    NiceMock<aruwsrc::mock::FireRateManagerMock> fireRateManager;
    MultiShotHandler multiShotHandler;
};

TEST_F(MultiShotHandlerTest, getShooterState_matches_setShooterState)
{
    for (uint8_t i = MultiShotHandler::SINGLE; i < MultiShotHandler::NUM_SHOOTER_STATES; i++)
    {
        multiShotHandler.setShooterState(static_cast<MultiShotHandler::ShooterState>(i));
        EXPECT_EQ(i, multiShotHandler.getShooterState());
    }
}

TEST_F(MultiShotHandlerTest, setShooterState_updates_cmdMapping)
{
    {
        InSequence seq;
        EXPECT_CALL(cmdMapping, setMaxTimesToSchedule(1));
        EXPECT_CALL(cmdMapping, setMaxTimesToSchedule(-1));
        EXPECT_CALL(cmdMapping, setMaxTimesToSchedule(-1));
    }

    {
        InSequence seq;
        EXPECT_CALL(fireRateManager, setFireRate(20));
        EXPECT_CALL(fireRateManager, setFireRate(10));
        EXPECT_CALL(fireRateManager, setFireRate(20));
    }

    multiShotHandler.setShooterState(MultiShotHandler::SINGLE);
    multiShotHandler.setShooterState(MultiShotHandler::FULL_AUTO_10HZ);
    multiShotHandler.setShooterState(MultiShotHandler::FULL_AUTO_20HZ);
}
