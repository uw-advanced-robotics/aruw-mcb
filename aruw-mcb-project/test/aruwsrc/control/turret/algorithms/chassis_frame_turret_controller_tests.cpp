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
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::mock;
using namespace testing;

#define RUN_SINGLE_PID_PITCH_0_CG()                      \
    ChassisFrameTurretController::runPitchPidController( \
        1,                                               \
        setpoint,                                        \
        0,                                               \
        0,                                               \
        0,                                               \
        &pid,                                            \
        &turretSubsystem);

#define SETUP_PITCH_TEST()                                                                      \
    aruwsrc::Drivers drivers;                                                                   \
    float setpoint = 0;                                                                         \
    ContiguousFloat currentPitchAngle(0, 0, 360);                                               \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers);                                    \
    tap::algorithms::SmoothPid pid(1, 0, 0, 0, 1, 1, 0, 1, 0);                                  \
    ON_CALL(turretSubsystem, getPitchSetpoint).WillByDefault(ReturnPointee(&setpoint));         \
    ON_CALL(turretSubsystem, getCurrentPitchValue).WillByDefault(ReturnRef(currentPitchAngle)); \
    ON_CALL(turretSubsystem, getPitchAngleFromCenter).WillByDefault(Return(0));                 \
    ON_CALL(turretSubsystem, getPitchVelocity).WillByDefault(Return(0));

TEST(
    ChassisFrameTurretController,
    runPitchPidController__pid_out_0_when_setpoints_match_p_controller)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchSetpoint(0));
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(90));
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(150));

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(0)).Times(3);

    setpoint = 0;
    currentPitchAngle.setValue(0);
    RUN_SINGLE_PID_PITCH_0_CG();

    setpoint = 90;
    currentPitchAngle.setValue(90);
    RUN_SINGLE_PID_PITCH_0_CG();

    setpoint = 150;
    currentPitchAngle.setValue(150);
    RUN_SINGLE_PID_PITCH_0_CG();
}

TEST(ChassisFrameTurretController, runPitchPidController__pid_out_positive_when_setpoint_gt_current)
{
    SETUP_PITCH_TEST();

    // setpoint > pitch angle, output should be > 0
    setpoint = 30;
    currentPitchAngle.setValue(20);
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(setpoint));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Gt(0)));

    RUN_SINGLE_PID_PITCH_0_CG();
}

TEST(ChassisFrameTurretController, runPitchPidController__pid_out_negative_when_setpoint_lt_current)
{
    SETUP_PITCH_TEST();

    // setpoint < pitch angle, output should be < 0
    setpoint = 30;
    currentPitchAngle.setValue(40);
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(setpoint));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Lt(0)));

    RUN_SINGLE_PID_PITCH_0_CG();
}

#define RUN_SINGLE_PID_YAW_0_CG() \
    ChassisFrameTurretController::runYawPidController(1, setpoint, &pid, &turretSubsystem);

#define SETUP_YAW_TEST()                                                                    \
    aruwsrc::Drivers drivers;                                                               \
    float setpoint = 0;                                                                     \
    ContiguousFloat currentYawAngle(0, 0, 360);                                             \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers);                                \
    tap::algorithms::SmoothPid pid(1, 0, 0, 0, 1, 1, 0, 1, 0);                              \
    ON_CALL(turretSubsystem, getYawSetpoint).WillByDefault(ReturnPointee(&setpoint));       \
    ON_CALL(turretSubsystem, getCurrentYawValue).WillByDefault(ReturnRef(currentYawAngle)); \
    ON_CALL(turretSubsystem, getYawVelocity).WillByDefault(Return(0));

TEST(ChassisFrameTurretController, runYawPidController__pid_out_0_when_setpoints_match_p_controller)
{
    SETUP_YAW_TEST();

    // Validate pitch setpoint set and pid output is reasonable
    EXPECT_CALL(turretSubsystem, setYawSetpoint(0));
    EXPECT_CALL(turretSubsystem, setYawSetpoint(90));
    EXPECT_CALL(turretSubsystem, setYawSetpoint(150));

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0)).Times(3);

    setpoint = 0;
    currentYawAngle.setValue(0);
    RUN_SINGLE_PID_YAW_0_CG();

    setpoint = 90;
    currentYawAngle.setValue(90);
    RUN_SINGLE_PID_YAW_0_CG();

    setpoint = 150;
    currentYawAngle.setValue(150);
    RUN_SINGLE_PID_YAW_0_CG();
}

TEST(ChassisFrameTurretController, runYawPidController__pid_out_positive_if_setpoint_gt_current)
{
    SETUP_YAW_TEST();

    // setpoint > pitch angle, output should be positive
    setpoint = 30;
    currentYawAngle.setValue(20);
    EXPECT_CALL(turretSubsystem, setYawSetpoint(setpoint));
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Gt(0)));
    RUN_SINGLE_PID_YAW_0_CG();
}

TEST(ChassisFrameTurretController, runYawPidController__pid_out_negative_if_setpoint_lt_current)
{
    SETUP_YAW_TEST();

    // setpoint < pitch angle, output should be < 0
    setpoint = 30;
    currentYawAngle.setValue(40);
    EXPECT_CALL(turretSubsystem, setYawSetpoint(setpoint));
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Lt(0)));
    RUN_SINGLE_PID_YAW_0_CG();
}
