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

#include "aruwsrc/control/turret/algorithms/turret_pid_turret_imu_world_rel.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace tap;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret::turret_imu_world_rel;
using namespace aruwsrc::mock;
using namespace testing;

TEST(TurretPidTurretImuWorldRel, transformChassisFrameToWorldFrame)
{
    // Chassis frame and world frame same
    EXPECT_FLOAT_EQ(0, transformChassisFrameToWorldFrame(0, 0, 0));
    EXPECT_FLOAT_EQ(10, transformChassisFrameToWorldFrame(0, 0, 10));
    EXPECT_FLOAT_EQ(100, transformChassisFrameToWorldFrame(0, 0, 100));
    EXPECT_FLOAT_EQ(200, transformChassisFrameToWorldFrame(0, 0, 200));
    // Chassis frame 0, world frame 90, chassis frame angle should be shifted by 90
    EXPECT_FLOAT_EQ(90, transformChassisFrameToWorldFrame(0, 90, 0));
    EXPECT_FLOAT_EQ(90 + 10, transformChassisFrameToWorldFrame(0, 90, 10));
    EXPECT_FLOAT_EQ(90 + 100, transformChassisFrameToWorldFrame(0, 90, 100));
    // Chassis frame static 90, world frame changing, angle to transform 90, shift transformed
    // angle based on how world frame changes.
    EXPECT_FLOAT_EQ(-90, transformChassisFrameToWorldFrame(90, 0, 0));
    EXPECT_FLOAT_EQ(-35, transformChassisFrameToWorldFrame(90, 10, 45));
    EXPECT_FLOAT_EQ(65, transformChassisFrameToWorldFrame(90, 20, 135));
}

TEST(TurretPidTurretImuWorldRel, transformWorldFrameValueToChassisFrame)
{
    // Chassis frame and world frame same
    EXPECT_FLOAT_EQ(0, transformWorldFrameValueToChassisFrame(0, 0, 0));
    EXPECT_FLOAT_EQ(10, transformWorldFrameValueToChassisFrame(0, 0, 10));
    EXPECT_FLOAT_EQ(100, transformWorldFrameValueToChassisFrame(0, 0, 100));
    EXPECT_FLOAT_EQ(200, transformWorldFrameValueToChassisFrame(0, 0, 200));
    // Chassis frame 0, world frame 90, world frame angle should be shifted by -90
    EXPECT_FLOAT_EQ(-90, transformWorldFrameValueToChassisFrame(0, 90, 0));
    EXPECT_FLOAT_EQ(-90 + 10, transformWorldFrameValueToChassisFrame(0, 90, 10));
    EXPECT_FLOAT_EQ(-90 + 100, transformWorldFrameValueToChassisFrame(0, 90, 100));
    // Chassis frame static 90, world frame changing, angle to transform 90, shift transformed
    // angle based on how world frame changes.
    EXPECT_FLOAT_EQ(90, transformWorldFrameValueToChassisFrame(90, 0, 0));
    EXPECT_FLOAT_EQ(125, transformWorldFrameValueToChassisFrame(90, 10, 45));
    EXPECT_FLOAT_EQ(205, transformWorldFrameValueToChassisFrame(90, 20, 135));
}

TEST(
    TurretPidTurretImuWorldRel,
    transforming_to_world_and_back_to_chassis_frame_preserves_original_value)
{
    // Chassis frame and world frame same
    EXPECT_FLOAT_EQ(
        0,
        transformChassisFrameToWorldFrame(0, 0, transformWorldFrameValueToChassisFrame(0, 0, 0)));
    EXPECT_FLOAT_EQ(
        10,
        transformChassisFrameToWorldFrame(0, 0, transformWorldFrameValueToChassisFrame(0, 0, 10)));
    EXPECT_FLOAT_EQ(
        100,
        transformChassisFrameToWorldFrame(0, 0, transformWorldFrameValueToChassisFrame(0, 0, 100)));
    EXPECT_FLOAT_EQ(
        200,
        transformChassisFrameToWorldFrame(0, 0, transformWorldFrameValueToChassisFrame(0, 0, 200)));
    EXPECT_FLOAT_EQ(
        0,
        transformChassisFrameToWorldFrame(0, 90, transformWorldFrameValueToChassisFrame(0, 90, 0)));
    EXPECT_FLOAT_EQ(
        10,
        transformChassisFrameToWorldFrame(
            0,
            90,
            transformWorldFrameValueToChassisFrame(0, 90, 10)));
    EXPECT_FLOAT_EQ(
        100,
        transformChassisFrameToWorldFrame(
            0,
            90,
            transformWorldFrameValueToChassisFrame(0, 90, 100)));
    EXPECT_FLOAT_EQ(
        0,
        transformChassisFrameToWorldFrame(90, 0, transformWorldFrameValueToChassisFrame(90, 0, 0)));
    EXPECT_FLOAT_EQ(
        45,
        transformChassisFrameToWorldFrame(
            90,
            10,
            transformWorldFrameValueToChassisFrame(90, 10, 45)));
    EXPECT_FLOAT_EQ(
        135,
        transformChassisFrameToWorldFrame(
            90,
            20,
            transformWorldFrameValueToChassisFrame(90, 20, 135)));
}

#define SETUP_YAW_TEST()                                                                    \
    Drivers drivers;                                                                        \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers);                                \
    float userInput = 0;                                                                    \
    ContiguousFloat worldFrameYawSetpoint(0, 0, 360);                                       \
    SmoothPid yawPid(1, 0, 0, 0, 1, 1, 0, 1, 0);                                            \
    ContiguousFloat currentYawValue(0, 0, 360);                                             \
    float imuYaw = 0;                                                                       \
    float imuGz = 0;                                                                        \
    bool yawLimited = false;                                                                \
    ON_CALL(turretSubsystem, getCurrentYawValue).WillByDefault(ReturnRef(currentYawValue)); \
    ON_CALL(drivers.turretMCBCanComm, getYaw).WillByDefault(ReturnPointee(&imuYaw));        \
    ON_CALL(drivers.turretMCBCanComm, getGz).WillByDefault(ReturnPointee(&imuGz));          \
    ON_CALL(turretSubsystem, yawLimited).WillByDefault(ReturnPointee(&yawLimited));

#define RUN_YAW_SINGLE_PID_CONTROLLER()  \
    runSinglePidYawWorldFrameController( \
        1,                               \
        userInput,                       \
        &drivers,                        \
        worldFrameYawSetpoint,           \
        yawPid,                          \
        &turretSubsystem);

TEST(
    TurretPidTurretImuWorldRel,
    runSinglePidYawWorldFrameController__chassis_not_moving_p_controller_output_reasonable)
{
    SETUP_YAW_TEST();

    // Simulate chassis not moving, so turret world and chassis frame are identical

    userInput = 0;
    imuYaw = 0;
    imuGz = 0;
    currentYawValue.setValue(0);
    yawLimited = false;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0)).Times(1);
    RUN_YAW_SINGLE_PID_CONTROLLER();

    // User input > current angle, output should be positive
    userInput = 90;
    currentYawValue.setValue(80);
    imuYaw = 80;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Gt(0))).Times(1);
    RUN_YAW_SINGLE_PID_CONTROLLER();

    // User input < current angle, output should be negative
    userInput = 100;
    currentYawValue.setValue(110);
    imuYaw = 110;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Lt(0))).Times(1);
    RUN_YAW_SINGLE_PID_CONTROLLER();
}

TEST(
    TurretPidTurretImuWorldRel,
    runSinglePidYawWorldFrameController__chassis_moves_p_controller_output_compensates_reasonably)
{
    SETUP_YAW_TEST();

    // Simulate chassis moving with setpoint staying at 90 degrees

    // yaw value 80, so chassis moved 10 degrees
    currentYawValue.setValue(80);
    // user input in world frame still equal to imu yaw, so output 0
    userInput = 90;
    imuYaw = 90;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0)).Times(1);
    RUN_YAW_SINGLE_PID_CONTROLLER();

    // yaw value 100, so chassis moved 10 degrees in other direction
    currentYawValue.setValue(100);
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0)).Times(1);
    RUN_YAW_SINGLE_PID_CONTROLLER();
}

#define SETUP_PITCH_TEST()                                                                      \
    Drivers drivers;                                                                            \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers);                                    \
    float userInput = 0;                                                                        \
    ContiguousFloat worldFramePitchSetpoint(0, 0, 360);                                         \
    SmoothPid pitchPid(1, 0, 0, 0, 1, 1, 0, 1, 0);                                              \
    ContiguousFloat currentPitchValue(0, 0, 360);                                               \
    float imuPitch = 0;                                                                         \
    float imuGx = 0;                                                                            \
    ON_CALL(turretSubsystem, getCurrentPitchValue).WillByDefault(ReturnRef(currentPitchValue)); \
    ON_CALL(turretSubsystem, getPitchAngleFromCenter).WillByDefault(Return(0));                 \
    ON_CALL(drivers.turretMCBCanComm, getPitch).WillByDefault(ReturnPointee(&imuPitch));        \
    ON_CALL(drivers.turretMCBCanComm, getGx).WillByDefault(ReturnPointee(&imuGx));

#define RUN_PITCH_SINGLE_PID_CONTROLLER()  \
    runSinglePidPitchWorldFrameController( \
        1,                                 \
        userInput,                         \
        &drivers,                          \
        0,                                 \
        0,                                 \
        0,                                 \
        worldFramePitchSetpoint,           \
        pitchPid,                          \
        &turretSubsystem);

TEST(TurretPidTurretImuWorldRel, runSinglePidPitchWorldFrameController__chassis_stationary)
{
    SETUP_PITCH_TEST();

    imuPitch = 90;
    userInput = 90;
    currentPitchValue.setValue(90);
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(Return(transformWorldFrameValueToChassisFrame(
            currentPitchValue.getValue(),
            imuPitch,
            userInput)));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(0)).Times(1);
    RUN_PITCH_SINGLE_PID_CONTROLLER();

    // Simulate moving turret, setpoint stays the same as does turret chassis base imu

    currentPitchValue.setValue(100);
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Lt(0))).Times(1);
    RUN_PITCH_SINGLE_PID_CONTROLLER();

    currentPitchValue.setValue(80);
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Gt(0))).Times(1);
    RUN_PITCH_SINGLE_PID_CONTROLLER();
}

TEST(TurretPidTurretImuWorldRel, runSinglePidPitchWorldFrameController__chassis_moving)
{
    SETUP_PITCH_TEST();

    userInput = 90;
    currentPitchValue.setValue(80);
    imuPitch = 90;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(Return(transformWorldFrameValueToChassisFrame(
            currentPitchValue.getValue(),
            imuPitch,
            userInput)));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(0)).Times(1);
    RUN_PITCH_SINGLE_PID_CONTROLLER();

    imuPitch = 90;
    currentPitchValue.setValue(100);
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(Return(transformWorldFrameValueToChassisFrame(
            currentPitchValue.getValue(),
            imuPitch,
            userInput)));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(0)).Times(1);
    RUN_PITCH_SINGLE_PID_CONTROLLER();
}
