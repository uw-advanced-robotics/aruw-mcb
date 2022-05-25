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

#include "tap/mock/command_mock.hpp"
#include "tap/mock/hold_repeat_command_mapping_mock.hpp"
#include "tap/mock/odometry_2d_interface_mock.hpp"

#include "aruwsrc/control/agitator/multi_shot_command_mapping.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/launch_speed_predictor_interface_mock.hpp"
#include "aruwsrc/mock/turret_cv_command_mock.hpp"

using namespace testing;
using namespace aruwsrc::agitator;
using namespace tap::communication::serial;

class MultiShotCommandMappingTest : public Test
{
protected:
    MultiShotCommandMappingTest()
        : matchingRMS(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP),
          turretCvCommand(&drivers, nullptr, nullptr, nullptr, odom, flywheels, 0, 0, 0),
          multiShotCommandMapping(
              drivers,
              singleLaunchCommand,
              fullAuto10HzCommand,
              fullAuto20HzCommand,
              turretCvCommand,
              matchingRMS)
    {
    }

    void SetUp() override {}

    tap::control::RemoteMapState matchingRMS;
    aruwsrc::Drivers drivers;
    tap::mock::CommandMock singleLaunchCommand;
    tap::mock::CommandMock fullAuto10HzCommand;
    tap::mock::CommandMock fullAuto20HzCommand;

private:
    tap::mock::Odometry2DInterfaceMock odom;
    aruwsrc::mock::LaunchSpeedPredictorInterfaceMock flywheels;

protected:
    aruwsrc::mock::TurretCVCommandMock turretCvCommand;
    MultiShotCommandMapping multiShotCommandMapping;
};

TEST_F(MultiShotCommandMappingTest, getShooterState_matches_setShooterState)
{
    for (uint8_t i = MultiShotCommandMapping::SINGLE;
         i < MultiShotCommandMapping::NUM_SHOOTER_STATES;
         i++)
    {
        multiShotCommandMapping.setShooterState(
            static_cast<MultiShotCommandMapping::ShooterState>(i));
        EXPECT_EQ(i, multiShotCommandMapping.getShooterState());
    }
}

TEST_F(MultiShotCommandMappingTest, setShooterState_switches_scheduled_command)
{
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&turretCvCommand))
        .WillByDefault(Return(false));

    InSequence seq;
    EXPECT_CALL(drivers.commandScheduler, addCommand(&singleLaunchCommand));
    EXPECT_CALL(drivers.commandScheduler, addCommand(&fullAuto10HzCommand));
    EXPECT_CALL(drivers.commandScheduler, addCommand(&fullAuto20HzCommand));

    multiShotCommandMapping.executeCommandMapping(matchingRMS);
    multiShotCommandMapping.setShooterState(MultiShotCommandMapping::FULL_AUTO_10HZ);
    multiShotCommandMapping.executeCommandMapping(matchingRMS);
    multiShotCommandMapping.setShooterState(MultiShotCommandMapping::FULL_AUTO_20HZ);
    multiShotCommandMapping.executeCommandMapping(matchingRMS);
}

TEST_F(
    MultiShotCommandMappingTest,
    executeCommandMapping_always_schedules_full_auto_when_cv_autolaunch_mode)
{
    ON_CALL(drivers.commandScheduler, isCommandScheduled(&turretCvCommand))
        .WillByDefault(Return(true));

    EXPECT_CALL(drivers.commandScheduler, addCommand(&fullAuto20HzCommand));

    multiShotCommandMapping.executeCommandMapping(matchingRMS);
}
