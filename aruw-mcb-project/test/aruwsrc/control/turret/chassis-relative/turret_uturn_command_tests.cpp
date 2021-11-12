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

#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/chassis-relative/turret_uturn_command.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc::mock;
using namespace aruwsrc::control::turret;
using namespace testing;
using tap::Drivers;

TEST(TurretUTurnCommand, isReady__return_true_when_turret_online)
{
    Drivers drivers;
    TurretSubsystemMock turret(&drivers);
    TurretUTurnCommand turretUturnCommand(&turret, 180);

    EXPECT_CALL(turret, isOnline).Times(2).WillOnce(Return(false)).WillOnce(Return(true));

    EXPECT_FALSE(turretUturnCommand.isReady());
    EXPECT_TRUE(turretUturnCommand.isReady());
}

TEST(TurretUTurnCommand, initialize__sets_turret_setpoint_based_on_specified_setpoint_offset)
{
    Drivers drivers;
    TurretSubsystemMock turret(&drivers);
    TurretSubsystemMock turret2(&drivers);
    TurretUTurnCommand turretUturnCommand180Deg(&turret, 180);
    TurretUTurnCommand turretUturnCommand90Deg(&turret2, 90);

    tap::algorithms::ContiguousFloat turretYawValue(0, 0, 360);
    tap::algorithms::ContiguousFloat turret2YawValue(45, 0, 360);

    EXPECT_CALL(turret, getCurrentYawValue).WillRepeatedly(ReturnRef(turretYawValue));
    EXPECT_CALL(turret, setYawSetpoint(180));
    EXPECT_CALL(turret2, getCurrentYawValue).WillRepeatedly(ReturnRef(turret2YawValue));
    EXPECT_CALL(turret2, setYawSetpoint(135));

    turretUturnCommand180Deg.initialize();
    turretUturnCommand90Deg.initialize();
}

TEST(TurretUTurnCommand, successfully_registers_with_scheduler)
{
    Drivers drivers;
    tap::control::CommandScheduler commandScheduler(&drivers, true);
    testing::NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretUTurnCommand turretUturnCommand(&turret, 180);

    EXPECT_CALL(turret, isOnline).WillOnce(Return(true));
    tap::algorithms::ContiguousFloat currentYawValue(0, 0, 360);
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(currentYawValue));

    commandScheduler.registerSubsystem(&turret);
    commandScheduler.addCommand(&turretUturnCommand);

    EXPECT_TRUE(commandScheduler.isCommandScheduled(&turretUturnCommand));
}

TEST(TurretUturnCOmmand, isFinished__always_return_true)
{
    Drivers drivers;
    tap::control::CommandScheduler commandScheduler(&drivers, true);
    testing::NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretUTurnCommand turretUturnCommand(&turret, 180);

    EXPECT_TRUE(turretUturnCommand.isFinished());
    EXPECT_TRUE(turretUturnCommand.isFinished());
    EXPECT_TRUE(turretUturnCommand.isFinished());
}
