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

#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
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
    ChassisFrameTurretControllerTest()
        : turretSubsystem(&drivers),
          setpoint(Angle(0)),
          currentAngle(Angle(0))
    {
    }

    void SetUp() override
    {
        ON_CALL(turretSubsystem.pitchMotor, getChassisFrameSetpoint)
            .WillByDefault(ReturnPointee(&setpoint));
        ON_CALL(turretSubsystem.yawMotor, getChassisFrameSetpoint)
            .WillByDefault(ReturnPointee(&setpoint));
        ON_CALL(turretSubsystem.pitchMotor, getChassisFrameMeasuredAngle)
            .WillByDefault(ReturnRef(currentAngle));
        ON_CALL(turretSubsystem.yawMotor, getChassisFrameMeasuredAngle)
            .WillByDefault(ReturnRef(currentAngle));
        ON_CALL(turretSubsystem.pitchMotor, getAngleFromCenter).WillByDefault(Return(0));
        ON_CALL(turretSubsystem.yawMotor, getAngleFromCenter).WillByDefault(Return(0));
        ON_CALL(turretSubsystem.pitchMotor, getChassisFrameVelocity).WillByDefault(Return(0));
        ON_CALL(turretSubsystem.yawMotor, getChassisFrameVelocity).WillByDefault(Return(0));
    }

    tap::Drivers drivers;
    NiceMock<TurretSubsystemMock> turretSubsystem;
    WrappedFloat setpoint;
    WrappedFloat currentAngle;
};

class PitchControllerTest : public ChassisFrameTurretControllerTest
{
protected:
    PitchControllerTest()
        : turretController(turretSubsystem.pitchMotor, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0})
    {
    }

    ChassisFramePitchTurretController turretController;
};

class YawControllerTest : public ChassisFrameTurretControllerTest
{
protected:
    YawControllerTest() : turretController(turretSubsystem.yawMotor, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0})
    {
    }

    ChassisFrameYawTurretController turretController;
};

TEST_F(PitchControllerTest, runPitchPidController_pid_out_0_when_setpoints_match_p_controller)
{
    // check setpoints 0, 90, 150
    EXPECT_CALL(turretSubsystem.pitchMotor, setChassisFrameSetpoint(0));
    EXPECT_CALL(turretSubsystem.pitchMotor, setChassisFrameSetpoint(M_PI_2));
    EXPECT_CALL(turretSubsystem.pitchMotor, setChassisFrameSetpoint(modm::toRadian(150)));

    // should set motor output to 0 for each setpoint
    EXPECT_CALL(
        turretSubsystem.pitchMotor,
        setMotorOutput(computeGravitationalForceOffset(
            TURRET_CG_X,
            TURRET_CG_Z,
            0,
            GRAVITY_COMPENSATION_SCALAR)))
        .Times(3);

    setpoint = Angle(0);
    currentAngle.setWrappedValue(0);
    turretController.runController(
        computeGravitationalForceOffset(TURRET_CG_X, TURRET_CG_Z, 0, GRAVITY_COMPENSATION_SCALAR),
        setpoint);

    setpoint = Angle(M_PI_2);
    currentAngle.setWrappedValue(M_PI_2);
    turretController.runController(1, setpoint);

    setpoint = Angle(modm::toRadian(150));
    currentAngle.setWrappedValue(modm::toRadian(150));
    turretController.runController(1, setpoint);
}

TEST_F(PitchControllerTest, runPitchPidController_pid_out_positive_when_setpoint_gt_current)
{
    // setpoint > pitch angle, output should be > 0
    setpoint = Angle(modm::toRadian(30));
    currentAngle.setWrappedValue(modm::toRadian(20));
    EXPECT_CALL(turretSubsystem.pitchMotor, setChassisFrameSetpoint(setpoint));
    EXPECT_CALL(
        turretSubsystem.pitchMotor,
        setMotorOutput(Gt(computeGravitationalForceOffset(
            TURRET_CG_X,
            TURRET_CG_Z,
            0,
            GRAVITY_COMPENSATION_SCALAR))));

    turretController.runController(1, setpoint);
}

TEST_F(PitchControllerTest, runPitchPidController_pid_out_negative_when_setpoint_lt_current)
{
    // setpoint < pitch angle, output should be < 0
    setpoint = Angle(modm::toRadian(30));
    currentAngle.setWrappedValue(modm::toRadian(40));
    EXPECT_CALL(turretSubsystem.pitchMotor, setChassisFrameSetpoint(setpoint));
    EXPECT_CALL(
        turretSubsystem.pitchMotor,
        setMotorOutput(Lt(computeGravitationalForceOffset(
            TURRET_CG_X,
            TURRET_CG_Z,
            0,
            GRAVITY_COMPENSATION_SCALAR))));

    turretController.runController(1, setpoint);
}

TEST_F(YawControllerTest, runYawPidController_pid_out_0_when_setpoints_match_p_controller)
{
    // Validate pitch setpoint set and pid output is reasonable
    EXPECT_CALL(turretSubsystem.yawMotor, setChassisFrameSetpoint(0));
    EXPECT_CALL(turretSubsystem.yawMotor, setChassisFrameSetpoint(M_PI_2));
    EXPECT_CALL(turretSubsystem.yawMotor, setChassisFrameSetpoint(modm::toRadian(150)));

    EXPECT_CALL(turretSubsystem.yawMotor, setMotorOutput(0)).Times(3);

    setpoint = Angle(0);
    currentAngle.setWrappedValue(0);
    turretController.runController(1, setpoint);

    setpoint = Angle(M_PI_2);
    currentAngle.setWrappedValue(M_PI_2);
    turretController.runController(1, setpoint);

    setpoint = Angle(modm::toRadian(150));
    currentAngle.setWrappedValue(modm::toRadian(150));
    turretController.runController(1, setpoint);
}

TEST_F(YawControllerTest, runYawPidController_pid_out_positive_if_setpoint_gt_current)
{
    // setpoint > pitch angle, output should be positive
    setpoint = Angle(modm::toRadian(30));
    currentAngle.setWrappedValue(modm::toRadian(20));
    EXPECT_CALL(turretSubsystem.yawMotor, setChassisFrameSetpoint(setpoint));
    EXPECT_CALL(turretSubsystem.yawMotor, setMotorOutput(Gt(0)));
    turretController.runController(1, setpoint);
}

TEST_F(YawControllerTest, runYawPidController_pid_out_negative_if_setpoint_lt_current)
{
    // setpoint < pitch angle, output should be < 0
    setpoint = Angle(modm::toRadian(30));
    currentAngle.setWrappedValue(modm::toRadian(40));
    EXPECT_CALL(turretSubsystem.yawMotor, setChassisFrameSetpoint(setpoint));
    EXPECT_CALL(turretSubsystem.yawMotor, setMotorOutput(Lt(0)));
    turretController.runController(1, setpoint);
}
