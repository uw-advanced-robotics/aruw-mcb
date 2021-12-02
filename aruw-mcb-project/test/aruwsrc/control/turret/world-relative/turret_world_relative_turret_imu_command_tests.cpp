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

#include "aruwsrc/control/turret/world-relative/turret_world_relative_turret_imu_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc::control::turret;
using namespace aruwsrc;
using namespace aruwsrc::mock;
using namespace testing;

TEST(TurretWorldRelativeTurretImuCommand, isReady__true_when_turret_online)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretWorldRelativeTurretImuCommand turretCmd(&drivers, &turret);

    ON_CALL(turret, isOnline).WillByDefault(Return(true));
    ON_CALL(drivers.turretMCBCanComm, isConnected).WillByDefault(Return(true));
    EXPECT_TRUE(turretCmd.isReady());

    ON_CALL(turret, isOnline).WillByDefault(Return(false));
    ON_CALL(drivers.turretMCBCanComm, isConnected).WillByDefault(Return(false));
    EXPECT_FALSE(turretCmd.isReady());
}

TEST(TurretWorldRelativeTurretImuCommand, isFinished__true_if_turret_offline)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretWorldRelativeTurretImuCommand turretCmd(&drivers, &turret);

    ON_CALL(turret, isOnline).WillByDefault(Return(true));
    ON_CALL(drivers.turretMCBCanComm, isConnected).WillByDefault(Return(true));
    EXPECT_FALSE(turretCmd.isFinished());

    ON_CALL(turret, isOnline).WillByDefault(Return(false));
    ON_CALL(drivers.turretMCBCanComm, isConnected).WillByDefault(Return(false));
    EXPECT_TRUE(turretCmd.isFinished());
}

TEST(TurretWorldRelativeTurretImuCommand, end__sets_desired_out_to_0)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretWorldRelativeTurretImuCommand turretCmd(&drivers, &turret);

    EXPECT_CALL(turret, setPitchMotorOutput).Times(2);
    EXPECT_CALL(turret, setYawMotorOutput).Times(2);

    turretCmd.end(true);
    turretCmd.end(false);
}
