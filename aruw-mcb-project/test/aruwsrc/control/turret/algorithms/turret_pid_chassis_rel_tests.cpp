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

#include "aruwsrc/control/turret/algorithms/turret_pid_chassis_rel.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace tap;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret::chassis_rel;
using namespace aruwsrc::mock;
using namespace testing;

#define RUN_SINGLE_PID_PITCH_0_CG() \
    runSinglePidPitchChassisFrameController(1, userInput, 0, 0, 0, pid, &turretSubsystem);

#define SETUP_PITCH_TEST()                                                                      \
    tap::Drivers drivers;                                                                       \
    float userInput = 0;                                                                        \
    ContiguousFloat currentPitchAngle(0, 0, 360);                                               \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers);                                    \
    tap::algorithms::SmoothPid pid(1, 0, 0, 0, 1, 1, 0, 1, 0);                                  \
    ON_CALL(turretSubsystem, getPitchSetpoint).WillByDefault(ReturnPointee(&userInput));        \
    ON_CALL(turretSubsystem, getCurrentPitchValue).WillByDefault(ReturnRef(currentPitchAngle)); \
    ON_CALL(turretSubsystem, getPitchAngleFromCenter).WillByDefault(Return(0));                 \
    ON_CALL(turretSubsystem, getPitchVelocity).WillByDefault(Return(0));

TEST(
    TurretPidChassisRel,
    runSinglePidPitchChassisFrameController__pid_out_0_when_setpoints_match_p_controller)
{
    SETUP_PITCH_TEST();

    // Validate pitch setpoint set and pid output is reasonable

    userInput = 0;
    currentPitchAngle.setValue(0);
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(userInput));
    RUN_SINGLE_PID_PITCH_0_CG();
    EXPECT_FLOAT_EQ(0, pid.getOutput());

    userInput = 90;
    currentPitchAngle.setValue(90);
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(userInput));
    RUN_SINGLE_PID_PITCH_0_CG();
    EXPECT_FLOAT_EQ(0, pid.getOutput());

    userInput = 150;
    currentPitchAngle.setValue(150);
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(userInput));
    RUN_SINGLE_PID_PITCH_0_CG();
    EXPECT_FLOAT_EQ(0, pid.getOutput());
}

TEST(
    TurretPidChassisRel,
    runSinglePidPitchChassisFrameController__pid_out_correct_sign_when_setpoints_dont_match_p_controller)
{
    SETUP_PITCH_TEST();

    // Validate pitch setpoint set and pid output is reasonable

    // pitch angle < setpoint, output should be < 0
    userInput = 30;
    currentPitchAngle.setValue(20);
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(userInput));
    RUN_SINGLE_PID_PITCH_0_CG();
    EXPECT_LT(0, pid.getOutput());

    // pitch angle > setpoint, output should be > 0
    userInput = 30;
    currentPitchAngle.setValue(40);
    EXPECT_CALL(turretSubsystem, setPitchSetpoint(userInput));
    RUN_SINGLE_PID_PITCH_0_CG();
    EXPECT_GT(0, pid.getOutput());
}

#define RUN_SINGLE_PID_YAW_0_CG() \
    runSinglePidYawChassisFrameController(1, userInput, pid, &turretSubsystem);

#define SETUP_YAW_TEST()                                                                    \
    tap::Drivers drivers;                                                                   \
    float userInput = 0;                                                                    \
    ContiguousFloat currentYawAngle(0, 0, 360);                                             \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers);                                \
    tap::algorithms::SmoothPid pid(1, 0, 0, 0, 1, 1, 0, 1, 0);                              \
    ON_CALL(turretSubsystem, getYawSetpoint).WillByDefault(ReturnPointee(&userInput));      \
    ON_CALL(turretSubsystem, getCurrentYawValue).WillByDefault(ReturnRef(currentYawAngle)); \
    ON_CALL(turretSubsystem, getYawVelocity).WillByDefault(Return(0));

TEST(
    TurretPidChassisRel,
    runSinglePidYawChassisFrameController__pid_out_0_when_setpoints_match_p_controller)
{
    SETUP_YAW_TEST();

    // Validate pitch setpoint set and pid output is reasonable

    userInput = 0;
    currentYawAngle.setValue(0);
    EXPECT_CALL(turretSubsystem, setYawSetpoint(userInput));
    RUN_SINGLE_PID_YAW_0_CG();
    EXPECT_FLOAT_EQ(0, pid.getOutput());

    userInput = 90;
    currentYawAngle.setValue(90);
    EXPECT_CALL(turretSubsystem, setYawSetpoint(userInput));
    RUN_SINGLE_PID_YAW_0_CG();
    EXPECT_FLOAT_EQ(0, pid.getOutput());

    userInput = 150;
    currentYawAngle.setValue(150);
    EXPECT_CALL(turretSubsystem, setYawSetpoint(userInput));
    RUN_SINGLE_PID_YAW_0_CG();
    EXPECT_FLOAT_EQ(0, pid.getOutput());
}

TEST(
    TurretPidChassisRel,
    runSinglePidYawChassisFrameController__pid_out_correct_sign_when_setpoints_dont_match_p_controller)
{
    SETUP_YAW_TEST();

    // Validate yaw setpoint set and pid output is reasonable

    // pitch angle < setpoint, output should be < 0
    userInput = 30;
    currentYawAngle.setValue(20);
    EXPECT_CALL(turretSubsystem, setYawSetpoint(userInput));
    RUN_SINGLE_PID_YAW_0_CG();
    EXPECT_LT(0, pid.getOutput());

    // pitch angle > setpoint, output should be > 0
    userInput = 30;
    currentYawAngle.setValue(40);
    EXPECT_CALL(turretSubsystem, setYawSetpoint(userInput));
    RUN_SINGLE_PID_YAW_0_CG();
    EXPECT_GT(0, pid.getOutput());
}
