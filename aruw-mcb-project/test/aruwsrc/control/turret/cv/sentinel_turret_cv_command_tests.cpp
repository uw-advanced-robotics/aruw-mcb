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

#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/cv/sentinel_turret_cv_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/agitator_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::turret::cv;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc::mock;
using namespace testing;

#define SETUP_TEST()                                                                       \
    Drivers drivers;                                                                       \
    NiceMock<TurretSubsystemMock> turret(&drivers);                                        \
    AgitatorSubsystemMock agitator(&drivers);                                              \
    ChassisFramePitchTurretController pitchController(&turret, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}); \
    ChassisFrameYawTurretController yawController(&turret, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0});     \
    SentinelTurretCVCommand turretCR(                                                      \
        &drivers,                                                                          \
        &turret,                                                                           \
        &agitator,                                                                         \
        &yawController,                                                                    \
        &pitchController);

TEST(SentinelTurretCVCommand, isReady_return_true_when_turret_online)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(true));

    EXPECT_TRUE(turretCR.isReady());
}

TEST(SentinelTurretCVCommand, isReady_return_false_when_turret_offline)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(false));

    EXPECT_FALSE(turretCR.isReady());
}

TEST(SentinelTurretCVCommand, isFinished_return_true_when_turret_offline)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(false));

    EXPECT_TRUE(turretCR.isFinished());
}

TEST(SentinelTurretCVCommand, isFinished_return_false_when_turret_online)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(true));

    EXPECT_FALSE(turretCR.isFinished());
}

TEST(SentinelTurretCVCommand, end_sets_motor_out_to_0)
{
    SETUP_TEST();

    EXPECT_CALL(turret, setPitchMotorOutput(0)).Times(2);
    EXPECT_CALL(turret, setYawMotorOutput(0)).Times(2);

    turretCR.end(true);
    turretCR.end(false);
}
