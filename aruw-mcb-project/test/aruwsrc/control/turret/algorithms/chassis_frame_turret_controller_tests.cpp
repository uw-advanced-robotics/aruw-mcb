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
#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/turret_controller_constants.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc::mock;
using namespace testing;

class ChassisFrameTurretControllerTest : public Test
{
protected:
    ChassisFrameTurretControllerTest() : turretSubsystem(&drivers), currentAngle(0, 0, 360) {}

    void SetUp() override
    {
        ON_CALL(turretSubsystem, getPitchSetpoint).WillByDefault(ReturnPointee(&setpoint));
        ON_CALL(turretSubsystem, getYawSetpoint).WillByDefault(ReturnPointee(&setpoint));
        ON_CALL(turretSubsystem, getCurrentPitchValue).WillByDefault(ReturnRef(currentAngle));
        ON_CALL(turretSubsystem, getCurrentYawValue).WillByDefault(ReturnRef(currentAngle));
        ON_CALL(turretSubsystem, getPitchAngleFromCenter).WillByDefault(Return(0));
        ON_CALL(turretSubsystem, getYawAngleFromCenter).WillByDefault(Return(0));
        ON_CALL(turretSubsystem, getPitchVelocity).WillByDefault(Return(0));
        ON_CALL(turretSubsystem, getYawVelocity).WillByDefault(Return(0));
    }

    aruwsrc::Drivers drivers;
    NiceMock<TurretSubsystemMock> turretSubsystem;
    float setpoint = 0;
    ContiguousFloat currentAngle;
};

class PitchControllerTest : public ChassisFrameTurretControllerTest
{
protected:
    PitchControllerTest() : turretController(&turretSubsystem, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}) {}

    ChassisFramePitchTurretController turretController;
};

class YawControllerTest : public ChassisFrameTurretControllerTest
{
protected:
    YawControllerTest() : turretController(&turretSubsystem, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}) {}

    ChassisFrameYawTurretController turretController;
};

TEST_F(PitchControllerTest, runPitchPidController_pid_out_0_when_setpoints_match_p_controller)
{
    // check setpoints 0, 90, 150
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(0));
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(90));
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(150));

    // should set motor output to 0 for each setpoint
    EXPECT_CALL(
        turretSubsystem,
        setPitchMotorOutput(computeGravitationalForceOffset(
            TURRET_CG_X,
            TURRET_CG_Z,
            0,
            GRAVITY_COMPENSATION_SCALAR)))
        .Times(3);

    setpoint = 0;
    currentAngle.setValue(0);
    turretController.runController(
        computeGravitationalForceOffset(TURRET_CG_X, TURRET_CG_Z, 0, GRAVITY_COMPENSATION_SCALAR),
        setpoint);

    setpoint = 90;
    currentAngle.setValue(90);
    turretController.runController(1, setpoint);

    setpoint = 150;
    currentAngle.setValue(150);
    turretController.runController(1, setpoint);
}

TEST_F(PitchControllerTest, runPitchPidController_pid_out_positive_when_setpoint_gt_current)
{
    // setpoint > pitch angle, output should be > 0
    setpoint = 30;
    currentAngle.setValue(20);
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(setpoint));
    EXPECT_CALL(
        turretSubsystem,
        setPitchMotorOutput(Gt(computeGravitationalForceOffset(
            TURRET_CG_X,
            TURRET_CG_Z,
            0,
            GRAVITY_COMPENSATION_SCALAR))));

    turretController.runController(1, setpoint);
}

TEST_F(PitchControllerTest, runPitchPidController_pid_out_negative_when_setpoint_lt_current)
{
    // setpoint < pitch angle, output should be < 0
    setpoint = 30;
    currentAngle.setValue(40);
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(setpoint));
    EXPECT_CALL(
        turretSubsystem,
        setPitchMotorOutput(Lt(computeGravitationalForceOffset(
            TURRET_CG_X,
            TURRET_CG_Z,
            0,
            GRAVITY_COMPENSATION_SCALAR))));

    turretController.runController(1, setpoint);
}

TEST_F(YawControllerTest, runYawPidController_pid_out_0_when_setpoints_match_p_controller)
{
    // Validate pitch setpoint set and pid output is reasonable
    EXPECT_CALL(turretSubsystem, setYawSetpoint(0));
    EXPECT_CALL(turretSubsystem, setYawSetpoint(90));
    EXPECT_CALL(turretSubsystem, setYawSetpoint(150));

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0)).Times(3);

    setpoint = 0;
    currentAngle.setValue(0);
    turretController.runController(1, setpoint);

    setpoint = 90;
    currentAngle.setValue(90);
    turretController.runController(1, setpoint);

    setpoint = 150;
    currentAngle.setValue(150);
    turretController.runController(1, setpoint);
}

TEST_F(YawControllerTest, runYawPidController_pid_out_positive_if_setpoint_gt_current)
{
    // setpoint > pitch angle, output should be positive
    setpoint = 30;
    currentAngle.setValue(20);
    EXPECT_CALL(turretSubsystem, setYawSetpoint(setpoint));
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Gt(0)));
    turretController.runController(1, setpoint);
}

TEST_F(YawControllerTest, runYawPidController_pid_out_negative_if_setpoint_lt_current)
{
    // setpoint < pitch angle, output should be < 0
    setpoint = 30;
    currentAngle.setValue(40);
    EXPECT_CALL(turretSubsystem, setYawSetpoint(setpoint));
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Lt(0)));
    turretController.runController(1, setpoint);
}
