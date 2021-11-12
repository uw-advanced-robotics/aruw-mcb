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

#include "aruwsrc/control/turret/world-relative/turret_world_relative_command.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc::control::turret;
using namespace tap;
using namespace aruwsrc::mock;
using namespace testing;

TEST(TurretWorldRelativeCommand, isReady__true_if_turret_online)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretWorldRelativeCommand turretCmd(&drivers, &turret);

    ON_CALL(turret, isOnline).WillByDefault(Return(true));
    EXPECT_TRUE(turretCmd.isReady());

    ON_CALL(turret, isOnline).WillByDefault(Return(false));
    EXPECT_FALSE(turretCmd.isReady());
}

TEST(TurretWorldRelativeCommand, isFinished__true_if_turret_offline)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretWorldRelativeCommand turretCmd(&drivers, &turret);

    ON_CALL(turret, isOnline).WillByDefault(Return(true));
    EXPECT_FALSE(turretCmd.isFinished());

    ON_CALL(turret, isOnline).WillByDefault(Return(false));
    EXPECT_TRUE(turretCmd.isFinished());
}

TEST(TurretWorldRelativeCommand, execute__runs_turret_wr_turret_imu_cmd_when_turret_imu_online)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretWorldRelativeCommand turretCmd(&drivers, &turret);

    tap::algorithms::ContiguousFloat currentYawValue(0, 0, 360);
    tap::algorithms::ContiguousFloat currentPitchValue(0, 0, 360);
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(currentYawValue));
    ON_CALL(turret, getCurrentPitchValue).WillByDefault(ReturnRef(currentYawValue));

    ON_CALL(drivers.turretMCBCanComm, isConnected).WillByDefault(Return(true));
    ON_CALL(turret, isOnline).WillByDefault(Return(true));

    // The turret MCB comm will be queried if the turret IMU command is running
    EXPECT_CALL(drivers.turretMCBCanComm, getYaw).Times(AtLeast(1));

    turretCmd.initialize();
    turretCmd.execute();
}

TEST(TurretWorldRelativeCommand, execute__runs_turret_wr_chassis_imu_cmd_when_turret_imu_offline)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretWorldRelativeCommand turretCmd(&drivers, &turret);

    tap::algorithms::ContiguousFloat currentYawValue(0, 0, 360);
    tap::algorithms::ContiguousFloat currentPitchValue(0, 0, 360);
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(currentYawValue));
    ON_CALL(turret, getCurrentPitchValue).WillByDefault(ReturnRef(currentYawValue));

    ON_CALL(drivers.turretMCBCanComm, isConnected).WillByDefault(Return(false));
    ON_CALL(turret, isOnline).WillByDefault(Return(true));

    // The turret MCB comm will be queried if the turret IMU command is running
    EXPECT_CALL(drivers.turretMCBCanComm, getYaw).Times(0);

    turretCmd.initialize();
    turretCmd.execute();
}

TEST(TurretWorldRelativeCommand, end__doesnt_set_des_out_when_no_cmds_scheduled)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretWorldRelativeCommand turretCmd(&drivers, &turret);

    ON_CALL(turret, isOnline).WillByDefault(Return(true));

    EXPECT_CALL(turret, setPitchMotorOutput(0)).Times(0);
    EXPECT_CALL(turret, setYawMotorOutput(0)).Times(0);

    turretCmd.end(true);
}

TEST(TurretWorldRelativeCommand, end__set_des_out_0_when_either_turret_command_initially_scheduled)
{
    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret(&drivers);
    TurretWorldRelativeCommand turretCmd(&drivers, &turret);

    tap::algorithms::ContiguousFloat currentYawValue(0, 0, 360);
    tap::algorithms::ContiguousFloat currentPitchValue(0, 0, 360);
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(currentYawValue));
    ON_CALL(turret, getCurrentPitchValue).WillByDefault(ReturnRef(currentYawValue));
    ON_CALL(turret, isOnline).WillByDefault(Return(true));

    EXPECT_CALL(turret, setPitchMotorOutput(0)).Times(2);
    EXPECT_CALL(turret, setYawMotorOutput(0)).Times(2);

    ON_CALL(drivers.turretMCBCanComm, isConnected).WillByDefault(Return(false));

    turretCmd.initialize();
    turretCmd.end(true);

    ON_CALL(drivers.turretMCBCanComm, isConnected).WillByDefault(Return(true));

    turretCmd.initialize();
    turretCmd.end(true);
}
