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

#include "aruwsrc/control/turret/user/turret_quick_turn_command.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc::mock;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::turret::user;
using namespace testing;
using tap::Drivers;

class TurretQuickTurnCommandTest : public Test
{
protected:
    TurretQuickTurnCommandTest() : turret(&drivers), turretUturnCommand(&turret, 180) {}

    tap::Drivers drivers;
    TurretSubsystemMock turret;
    TurretQuickTurnCommand turretUturnCommand;
};

TEST_F(TurretQuickTurnCommandTest, isReady_return_true_when_turret_online)
{
    EXPECT_CALL(turret.yawMotor, isOnline).Times(2).WillOnce(Return(false)).WillOnce(Return(true));

    EXPECT_FALSE(turretUturnCommand.isReady());
    EXPECT_TRUE(turretUturnCommand.isReady());
}

TEST(TurretQuickTurnCommand, initialize_sets_turret_setpoint_based_on_specified_setpoint_offset)
{
    tap::Drivers drivers;
    TurretSubsystemMock turret(&drivers);
    TurretSubsystemMock turret2(&drivers);
    TurretQuickTurnCommand turretUturnCommand180Deg(&turret, M_PI);
    TurretQuickTurnCommand turretUturnCommand90Deg(&turret2, M_PI_2);

    tap::algorithms::WrappedFloat turretYawValue(0, 0, M_TWOPI);
    tap::algorithms::WrappedFloat turret2YawValue(M_PI_4, 0, M_TWOPI);

    EXPECT_CALL(turret.yawMotor, attachTurretController(nullptr));
    ON_CALL(turret.yawMotor, getChassisFrameMeasuredAngle).WillByDefault(ReturnRef(turretYawValue));
    EXPECT_CALL(turret.yawMotor, setChassisFrameSetpoint(M_PI));

    EXPECT_CALL(turret2.yawMotor, attachTurretController(nullptr));
    ON_CALL(turret2.yawMotor, getChassisFrameMeasuredAngle)
        .WillByDefault(ReturnRef(turret2YawValue));
    EXPECT_CALL(turret2.yawMotor, setChassisFrameSetpoint(M_PI_4 + M_PI_2));

    turretUturnCommand180Deg.initialize();
    turretUturnCommand90Deg.initialize();
}

TEST_F(TurretQuickTurnCommandTest, successfully_registers_with_scheduler)
{
    tap::control::CommandScheduler commandScheduler(&drivers, true);

    EXPECT_CALL(turret.yawMotor, isOnline).WillOnce(Return(true));
    tap::algorithms::WrappedFloat currentYawValue(0, 0, M_TWOPI);
    ON_CALL(turret.yawMotor, getChassisFrameMeasuredAngle)
        .WillByDefault(ReturnRef(currentYawValue));

    commandScheduler.registerSubsystem(&turret);
    commandScheduler.addCommand(&turretUturnCommand);

    EXPECT_TRUE(commandScheduler.isCommandScheduled(&turretUturnCommand));
}

TEST_F(TurretQuickTurnCommandTest, isFinished_always_return_true)
{
    EXPECT_TRUE(turretUturnCommand.isFinished());
    EXPECT_TRUE(turretUturnCommand.isFinished());
    EXPECT_TRUE(turretUturnCommand.isFinished());
}
