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
#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/user/turret_user_world_relative_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::turret::user;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc;
using namespace aruwsrc::mock;
using namespace testing;

#define SETUP_TEST()                                                                       \
    Drivers drivers;                                                                       \
    NiceMock<TurretSubsystemMock> turret(&drivers);                                        \
    ChassisFramePitchTurretController chassisFramePitchTurretController(                   \
        &turret,                                                                           \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0});                                                   \
    WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController(             \
        &drivers,                                                                          \
        &turret,                                                                           \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0});                                                   \
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController( \
        &drivers,                                                                          \
        &turret,                                                                           \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0},                                                    \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0});                                                   \
                                                                                           \
    WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController(     \
        &drivers,                                                                          \
        &turret,                                                                           \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0},                                                    \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0});                                                   \
    TurretUserWorldRelativeCommand turretCmd(                                              \
        &drivers,                                                                          \
        &turret,                                                                           \
        &worldFrameYawChassisImuController,                                                \
        &chassisFramePitchTurretController,                                                \
        &worldFrameYawTurretImuController,                                                 \
        &worldFramePitchTurretImuController);

TEST(TurretUserWorldRelativeCommand, isReady_true_if_turret_online)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(true));
    EXPECT_TRUE(turretCmd.isReady());

    ON_CALL(turret, isOnline).WillByDefault(Return(false));
    EXPECT_FALSE(turretCmd.isReady());
}

TEST(TurretUserWorldRelativeCommand, isFinished_true_if_turret_offline)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(true));
    EXPECT_FALSE(turretCmd.isFinished());

    ON_CALL(turret, isOnline).WillByDefault(Return(false));
    EXPECT_TRUE(turretCmd.isFinished());
}

TEST(TurretUserWorldRelativeCommand, execute_runs_turret_wr_turret_imu_cmd_when_turret_imu_online)
{
    SETUP_TEST();

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

TEST(TurretUserWorldRelativeCommand, execute_runs_turret_wr_chassis_imu_cmd_when_turret_imu_offline)
{
    SETUP_TEST();

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

TEST(TurretUserWorldRelativeCommand, end_doesnt_set_des_out_when_no_cmds_scheduled)
{
    SETUP_TEST();

    ON_CALL(turret, isOnline).WillByDefault(Return(true));

    EXPECT_CALL(turret, setPitchMotorOutput(0)).Times(0);
    EXPECT_CALL(turret, setYawMotorOutput(0)).Times(0);

    turretCmd.end(true);
}

TEST(
    TurretUserWorldRelativeCommand,
    end_set_des_out_0_when_either_turret_command_initially_scheduled)
{
    SETUP_TEST();

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
