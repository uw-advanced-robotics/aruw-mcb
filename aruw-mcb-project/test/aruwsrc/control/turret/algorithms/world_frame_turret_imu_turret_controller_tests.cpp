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

#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::mock;
using namespace testing;

TEST(WorldFrameTurretImuTurretController, transformChassisFrameToWorldFrame)
{
    // Chassis frame and world frame same
    EXPECT_FLOAT_EQ(
        0,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(0, 0, 0));
    EXPECT_FLOAT_EQ(
        10,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(0, 0, 10));
    EXPECT_FLOAT_EQ(
        100,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(0, 0, 100));
    EXPECT_FLOAT_EQ(
        200,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(0, 0, 200));
    // Chassis frame 0, world frame 90, chassis frame angle should be shifted by 90
    EXPECT_FLOAT_EQ(
        90,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(0, 90, 0));
    EXPECT_FLOAT_EQ(
        90 + 10,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(0, 90, 10));
    EXPECT_FLOAT_EQ(
        90 + 100,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(0, 90, 100));
    // Chassis frame static 90, world frame changing, angle to transform 90, shift transformed
    // angle based on how world frame changes.
    EXPECT_FLOAT_EQ(
        -90,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(90, 0, 0));
    EXPECT_FLOAT_EQ(
        -35,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(90, 10, 45));
    EXPECT_FLOAT_EQ(
        65,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(90, 20, 135));
}

TEST(WorldFrameTurretImuTurretController, transformWorldFrameValueToChassisFrame)
{
    // Chassis frame and world frame same
    EXPECT_FLOAT_EQ(
        0,
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(0, 0, 0));
    EXPECT_FLOAT_EQ(
        10,
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(0, 0, 10));
    EXPECT_FLOAT_EQ(
        100,
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(0, 0, 100));
    EXPECT_FLOAT_EQ(
        200,
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(0, 0, 200));
    // Chassis frame 0, world frame 90, world frame angle should be shifted by -90
    EXPECT_FLOAT_EQ(
        -90,
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(0, 90, 0));
    EXPECT_FLOAT_EQ(
        -90 + 10,
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(0, 90, 10));
    EXPECT_FLOAT_EQ(
        -90 + 100,
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(0, 90, 100));
    // Chassis frame static 90, world frame changing, angle to transform 90, shift transformed
    // angle based on how world frame changes.
    EXPECT_FLOAT_EQ(
        90,
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(90, 0, 0));
    EXPECT_FLOAT_EQ(
        125,
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(90, 10, 45));
    EXPECT_FLOAT_EQ(
        205,
        WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(90, 20, 135));
}

TEST(
    WorldFrameTurretImuTurretController,
    transforming_to_world_and_back_to_chassis_frame_preserves_original_value)
{
    // Chassis frame and world frame same
    EXPECT_FLOAT_EQ(
        0,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            0,
            0,
            WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(0, 0, 0)));
    EXPECT_FLOAT_EQ(
        10,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            0,
            0,
            WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(0, 0, 10)));
    EXPECT_FLOAT_EQ(
        100,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            0,
            0,
            WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                0,
                0,
                100)));
    EXPECT_FLOAT_EQ(
        200,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            0,
            0,
            WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                0,
                0,
                200)));
    EXPECT_FLOAT_EQ(
        0,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            0,
            90,
            WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(0, 90, 0)));
    EXPECT_FLOAT_EQ(
        10,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            0,
            90,
            WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                0,
                90,
                10)));
    EXPECT_FLOAT_EQ(
        100,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            0,
            90,
            WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                0,
                90,
                100)));
    EXPECT_FLOAT_EQ(
        0,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            90,
            0,
            WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(90, 0, 0)));
    EXPECT_FLOAT_EQ(
        45,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            90,
            10,
            WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                90,
                10,
                45)));
    EXPECT_FLOAT_EQ(
        135,
        WorldFrameTurretImuTurretController::transformChassisFrameToWorldFrame(
            90,
            20,
            WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                90,
                20,
                135)));
}

#define SETUP_YAW_TEST()                                                                    \
    Drivers drivers;                                                                        \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers);                                \
    float turretSetpoint = 0;                                                               \
    ContiguousFloat worldFrameYawSetpoint(0, 0, 360);                                       \
    SmoothPid yawPid(1, 0, 0, 0, 1, 1, 0, 1, 0);                                            \
    ContiguousFloat currentYawValue(0, 0, 360);                                             \
    float imuYaw = 0;                                                                       \
    float imuGz = 0;                                                                        \
    bool yawLimited = false;                                                                \
    ON_CALL(turretSubsystem, getCurrentYawValue).WillByDefault(ReturnRef(currentYawValue)); \
    ON_CALL(drivers.turretMCBCanComm, getYaw).WillByDefault(ReturnPointee(&imuYaw));        \
    ON_CALL(drivers.turretMCBCanComm, getYawVelocity).WillByDefault(ReturnPointee(&imuGz)); \
    ON_CALL(turretSubsystem, yawLimited).WillByDefault(ReturnPointee(&yawLimited));

#define RUN_YAW_SINGLE_PID_CONTROLLER()                       \
    WorldFrameTurretImuTurretController::runYawPidController( \
        drivers,                                              \
        1,                                                    \
        turretSetpoint,                                       \
        &worldFrameYawSetpoint,                               \
        &yawPid,                                              \
        &turretSubsystem);

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController__chassis_not_moving_setpoint_actual_identical_0_out)
{
    SETUP_YAW_TEST();

    // Simulate chassis not moving, so turret world and chassis frame are identical

    turretSetpoint = 0;
    imuYaw = 0;
    imuGz = 0;
    currentYawValue.setValue(0);
    yawLimited = false;

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0));

    RUN_YAW_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController__chassis_not_moving_setpoint_gt_actual_output_positive)
{
    SETUP_YAW_TEST();

    // Simulate chassis not moving, so turret world and chassis frame are identical

    imuGz = 0;
    yawLimited = false;

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Gt(0)));

    // User input > current angle, output should be positive
    turretSetpoint = 90;
    currentYawValue.setValue(80);
    imuYaw = 80;
    RUN_YAW_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController__chassis_not_moving_setpoint_lt_actual_output_negative)
{
    SETUP_YAW_TEST();

    // Simulate chassis not moving, so turret world and chassis frame are identical

    imuGz = 0;
    yawLimited = false;

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Lt(0)));

    // Setpoint < current angle, output should be negative
    turretSetpoint = 100;
    currentYawValue.setValue(110);
    imuYaw = 110;
    RUN_YAW_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController__chassis_rotated_positive_direction_world_setpoint_and_actual_equal_0_output)
{
    SETUP_YAW_TEST();

    // Simulate chassis moving with setpoint staying at 90 degrees

    // chassis frame yaw value 80, so chassis moved +10 degrees
    currentYawValue.setValue(80);
    // user input in world frame still equal to imu yaw, so output 0
    turretSetpoint = 90;
    imuYaw = 90;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0));

    RUN_YAW_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController__chassis_rotated_negative_direction_world_setpoint_and_actual_equal_0_output)
{
    SETUP_YAW_TEST();

    // Simulate chassis moving with setpoint staying at 90 degrees

    // yaw value 100, so chassis moved -10 degrees
    currentYawValue.setValue(100);
    turretSetpoint = 90;
    imuYaw = 90;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0));

    RUN_YAW_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController__chassis_rotated_positive_direction_world_setpoint_lt_actual_negative_output)
{
    SETUP_YAW_TEST();

    currentYawValue.setValue(90);
    turretSetpoint = 90;
    imuYaw = 100;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Lt(0)));

    RUN_YAW_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController__chassis_rotated_negative_direction_world_setpoint_gt_actual_positive_output)
{
    SETUP_YAW_TEST();

    currentYawValue.setValue(90);
    turretSetpoint = 90;
    imuYaw = 80;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Gt(0)));

    RUN_YAW_SINGLE_PID_CONTROLLER();
}

#define SETUP_PITCH_TEST()                                                                      \
    Drivers drivers;                                                                            \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers);                                    \
    float turretSetpoint = 0;                                                                   \
    ContiguousFloat worldFramePitchSetpoint(0, 0, 360);                                         \
    SmoothPid pitchPid(1, 0, 0, 0, 1, 1, 0, 1, 0);                                              \
    ContiguousFloat currentPitchValue(0, 0, 360);                                               \
    float imuPitch = 0;                                                                         \
    float imuGx = 0;                                                                            \
    ON_CALL(turretSubsystem, getCurrentPitchValue).WillByDefault(ReturnRef(currentPitchValue)); \
    ON_CALL(turretSubsystem, getPitchAngleFromCenter).WillByDefault(Return(0));                 \
    ON_CALL(drivers.turretMCBCanComm, getPitch).WillByDefault(ReturnPointee(&imuPitch));        \
    ON_CALL(drivers.turretMCBCanComm, getPitchVelocity).WillByDefault(ReturnPointee(&imuGx));

#define RUN_PITCH_SINGLE_PID_CONTROLLER()                       \
    WorldFrameTurretImuTurretController::runPitchPidController( \
        drivers,                                                \
        1,                                                      \
        turretSetpoint,                                         \
        0,                                                      \
        0,                                                      \
        0,                                                      \
        &worldFramePitchSetpoint,                               \
        &pitchPid,                                              \
        &turretSubsystem);

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController__chassis_stationary_world_frame_stationary_output_0)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(0));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Lt(0)));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Gt(0)));

    imuPitch = 90;
    turretSetpoint = 90;
    currentPitchValue.setValue(90);
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(
            Return(WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                currentPitchValue.getValue(),
                imuPitch,
                turretSetpoint)));
    RUN_PITCH_SINGLE_PID_CONTROLLER();

    // Simulate moving turret, setpoint stays the same as does turret chassis base imu

    currentPitchValue.setValue(100);
    RUN_PITCH_SINGLE_PID_CONTROLLER();

    currentPitchValue.setValue(80);
    RUN_PITCH_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController__chassis_tilted_up_world_setpoint_gt_actual_output_positive)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Gt(0)));

    turretSetpoint = 110;
    currentPitchValue.setValue(90);
    imuPitch = 90;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(
            Return(WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                currentPitchValue.getValue(),
                imuPitch,
                turretSetpoint)));

    RUN_PITCH_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController__chassis_tilted_up_world_setpoint_lt_actual_output_negative)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Lt(0)));

    turretSetpoint = 80;
    currentPitchValue.setValue(90);
    imuPitch = 90;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(
            Return(WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                currentPitchValue.getValue(),
                imuPitch,
                turretSetpoint)));

    RUN_PITCH_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController__chassis_tilted_down_world_setpoint_gt_actual_output_positive)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Gt(0)));

    turretSetpoint = 90;
    currentPitchValue.setValue(90);
    imuPitch = -10;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(
            Return(WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                currentPitchValue.getValue(),
                imuPitch,
                turretSetpoint)));

    RUN_PITCH_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController__chassis_tilted_down_world_setpoint_lt_actual_output_negative)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Lt(0)));

    turretSetpoint = 70;
    currentPitchValue.setValue(90);
    imuPitch = 90;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(
            Return(WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                currentPitchValue.getValue(),
                imuPitch,
                turretSetpoint)));

    RUN_PITCH_SINGLE_PID_CONTROLLER();
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController__chassis_moved_world_frame_stationary_output_0)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(0)).Times(2);

    turretSetpoint = 90;
    currentPitchValue.setValue(80);
    imuPitch = 90;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(
            Return(WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                currentPitchValue.getValue(),
                imuPitch,
                turretSetpoint)));
    RUN_PITCH_SINGLE_PID_CONTROLLER();

    imuPitch = 90;
    currentPitchValue.setValue(100);
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(
            Return(WorldFrameTurretImuTurretController::transformWorldFrameValueToChassisFrame(
                currentPitchValue.getValue(),
                imuPitch,
                turretSetpoint)));
    RUN_PITCH_SINGLE_PID_CONTROLLER();
}
