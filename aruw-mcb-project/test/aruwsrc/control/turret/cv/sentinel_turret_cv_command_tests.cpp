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

#include "aruwsrc/control/turret/cv/sentinel_turret_cv_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/agitator_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::mock;
using namespace testing;

TEST(SentinelTurretCVCommand, isReady__return_true_when_turret_online)
{
    Drivers drivers;
    TurretSubsystemMock turret(&drivers);
    AgitatorSubsystemMock agitator(&drivers);
    SentinelTurretCVCommand turretCR(&drivers, &turret, &agitator);

    EXPECT_CALL(turret, isOnline).WillOnce(Return(true)).WillOnce(Return(false));

    EXPECT_TRUE(turretCR.isReady());
    EXPECT_FALSE(turretCR.isReady());
}

TEST(SentinelTurretCVCommand, isFinished__return_true_when_turret_offline)
{
    Drivers drivers;
    TurretSubsystemMock turret(&drivers);
    AgitatorSubsystemMock agitator(&drivers);
    SentinelTurretCVCommand turretCR(&drivers, &turret, &agitator);

    EXPECT_CALL(turret, isOnline).WillOnce(Return(false)).WillOnce(Return(true));

    EXPECT_TRUE(turretCR.isFinished());
    EXPECT_FALSE(turretCR.isFinished());
}

TEST(SentinelTurretCVCommand, end__sets_motor_out_to_0)
{
    Drivers drivers;
    TurretSubsystemMock turret(&drivers);
    AgitatorSubsystemMock agitator(&drivers);
    SentinelTurretCVCommand turretCR(&drivers, &turret, &agitator);

    EXPECT_CALL(turret, setPitchMotorOutput(0)).Times(2);
    EXPECT_CALL(turret, setYawMotorOutput(0)).Times(2);

    turretCR.end(true);
    turretCR.end(false);
}
