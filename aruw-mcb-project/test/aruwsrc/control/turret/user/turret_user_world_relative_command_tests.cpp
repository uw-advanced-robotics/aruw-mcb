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
using namespace tap::algorithms;
using namespace aruwsrc::mock;
using namespace testing;



// static float computeValidMeasurementSetpointError(const TurretMotor &motor,
// float setpoint, float measurement) {
//     // motor.get

//     tap::algorithms::ContiguousFloat setpointMeasurementDiff(setpoint, 0, M_TWOPI);
//     setpointMeasurementDiff.difference(measurement);

//     // Get chassis frame setpoint

//     return 0;
// }


class TurretUserWorldRelativeCommandTest : public Test
{
protected:
    TurretUserWorldRelativeCommandTest()
        : turret(&drivers),
          chassisFramePitchTurretController(&turret.pitchMotor, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}),
          worldFrameYawChassisImuController(
              &drivers,
              &turret.yawMotor,
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}),
          worldFramePitchTurretImuController(
              &drivers,
              &turret.pitchMotor,
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0},
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}),
          worldFrameYawTurretImuController(
              &drivers,
              &turret.yawMotor,
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0},
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}),
          turretCmd(
              &drivers,
              &turret,
              &worldFrameYawChassisImuController,
              &chassisFramePitchTurretController,
              &worldFrameYawTurretImuController,
              &worldFramePitchTurretImuController,
              1,
              1),
          currentYawValue(0, 0, M_TWOPI),
          currentPitchValue(0, 0, M_TWOPI),
          yawSetpoint(0, 0, M_TWOPI),
          pitchSetpoint(0, 0, M_TWOPI)
    {
    }

    void SetUp() override
    {
        ON_CALL(turret.yawMotor, getChassisFrameMeasuredAngle)
            .WillByDefault(ReturnRef(currentYawValue));
        ON_CALL(turret.pitchMotor, getChassisFrameMeasuredAngle)
            .WillByDefault(ReturnRef(currentPitchValue));
        ON_CALL(turret.yawMotor, isOnline).WillByDefault(ReturnPointee(&turretOnline));
        ON_CALL(turret.pitchMotor, isOnline).WillByDefault(ReturnPointee(&turretOnline));
        ON_CALL(drivers.turretMCBCanComm, isConnected)
            .WillByDefault(ReturnPointee(&turretMcbCanCommConnected));
        ON_CALL(turret.yawMotor, getChassisFrameSetpoint).WillByDefault(ReturnRef(yawSetpoint));
        ON_CALL(turret.pitchMotor, getChassisFrameSetpoint).WillByDefault(ReturnRef(pitchSetpoint));
        ON_CALL(turret.yawMotor, getConfig).WillByDefault(ReturnRef(config));
        ON_CALL(turret.pitchMotor, getConfig).WillByDefault(ReturnRef(config));
    }

    Drivers drivers;
    NiceMock<TurretSubsystemMock> turret;
    ChassisFramePitchTurretController chassisFramePitchTurretController;
    WorldFrameYawChassisImuTurretController worldFrameYawChassisImuController;
    WorldFramePitchTurretImuCascadePidTurretController worldFramePitchTurretImuController;
    WorldFrameYawTurretImuCascadePidTurretController worldFrameYawTurretImuController;
    TurretUserWorldRelativeCommand turretCmd;
    ContiguousFloat currentYawValue;
    ContiguousFloat currentPitchValue;
    ContiguousFloat yawSetpoint;
    ContiguousFloat pitchSetpoint;
    bool turretOnline = false;
    bool turretMcbCanCommConnected = false;
    TurretMotorConfig config = {};
};

TEST_F(TurretUserWorldRelativeCommandTest, isReady_true_if_turret_online_isFinished_opposite)
{
    turretOnline = true;
    EXPECT_TRUE(turretCmd.isReady());
    EXPECT_FALSE(turretCmd.isFinished());

    turretOnline = false;
    EXPECT_FALSE(turretCmd.isReady());
    EXPECT_TRUE(turretCmd.isFinished());
}

TEST_F(
    TurretUserWorldRelativeCommandTest,
    execute_runs_turret_wr_turret_imu_cmd_when_turret_imu_online)
{
    turretMcbCanCommConnected = true;
    turretOnline = true;

    // The turret MCB comm will be queried if the turret IMU command is running
    EXPECT_CALL(drivers.turretMCBCanComm, getYaw).Times(AtLeast(1));

    turretCmd.initialize();
    turretCmd.execute();
}

TEST_F(
    TurretUserWorldRelativeCommandTest,
    execute_runs_turret_wr_chassis_imu_cmd_when_turret_imu_offline)
{
    turretMcbCanCommConnected = false;
    turretOnline = true;

    // The turret MCB comm will be queried if the turret IMU command is running
    EXPECT_CALL(drivers.turretMCBCanComm, getYaw).Times(0);

    turretCmd.initialize();
    turretCmd.execute();
}

TEST_F(TurretUserWorldRelativeCommandTest, end_doesnt_set_des_out_when_no_cmds_scheduled)
{
    turretOnline = true;

    EXPECT_CALL(turret.yawMotor, setMotorOutput(0)).Times(0);
    EXPECT_CALL(turret.pitchMotor, setMotorOutput(0)).Times(0);

    turretCmd.end(true);
}

TEST_F(
    TurretUserWorldRelativeCommandTest,
    end_set_des_out_0_when_either_turret_command_initially_scheduled)
{
    turretOnline = true;

    EXPECT_CALL(turret.pitchMotor, setMotorOutput(0)).Times(2);
    EXPECT_CALL(turret.yawMotor, setMotorOutput(0)).Times(2);

    turretMcbCanCommConnected = false;
    turretCmd.initialize();
    turretCmd.end(true);

    turretMcbCanCommConnected = true;
    turretCmd.initialize();
    turretCmd.end(true);
}
