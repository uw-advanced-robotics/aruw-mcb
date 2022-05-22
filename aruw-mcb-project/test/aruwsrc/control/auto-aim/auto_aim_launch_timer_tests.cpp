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
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"
#include "aruwsrc/mock/otto_ballistics_solver_mock.hpp"

#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/drivers.hpp"

using namespace testing;
using namespace aruwsrc::serial;
using namespace aruwsrc::control::auto_aim;

class AutoAimLaunchTimerTest : public Test
{
protected:
    AutoAimLaunchTimerTest();

    void SetUp() override {}

    NiceMock<aruwsrc::mock::VisionCoprocessorMock> visionCoprocessor;
    NiceMock<aruwsrc::mock::OttoBallisticsSolverMock> ballistics;
};

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_no_target_from_coprocessor_gives_no_target_inclination)
{
    VisionCoprocessor::TurretAimData aimData = {0};
    aimData.hasTarget = 0;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0))
        .WillOnce(ReturnPointee(&aimData));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::NO_TARGET, result);
}

// TEST_F(MultiShotHandlerTest, setShooterState_updates_cmdMapping)
// {
//     InSequence seq;
//     EXPECT_CALL(cmdMapping, setMaxTimesToSchedule(1));
//     EXPECT_CALL(cmdMapping, setMaxTimesToSchedule(3));
//     EXPECT_CALL(cmdMapping, setMaxTimesToSchedule(-1));

//     multiShotHandler.setShooterState(MultiShotHandler::SINGLE);
//     multiShotHandler.setShooterState(MultiShotHandler::BURST);
//     multiShotHandler.setShooterState(MultiShotHandler::FULL_AUTO);
// }
