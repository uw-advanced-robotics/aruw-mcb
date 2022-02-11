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

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc::mock;
using namespace testing;

#define SETUP_YAW_TEST()                                                                    \
    Drivers drivers;                                                                        \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers);                                \
    WorldFrameYawTurretImuCascadePidTurretController turretController(                      \
        &drivers,                                                                           \
        &turretSubsystem,                                                                   \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0},                                                     \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0});                                                    \
    float turretSetpoint = 0;                                                               \
    ContiguousFloat worldFrameYawSetpoint(0, 0, 360);                                       \
    ContiguousFloat currentYawValue(0, 0, 360);                                             \
    float imuYaw = 0;                                                                       \
    float imuGz = 0;                                                                        \
    bool yawLimited = false;                                                                \
    ON_CALL(turretSubsystem, getCurrentYawValue).WillByDefault(ReturnRef(currentYawValue)); \
    ON_CALL(drivers.turretMCBCanComm, getYaw).WillByDefault(ReturnPointee(&imuYaw));        \
    ON_CALL(drivers.turretMCBCanComm, getYawVelocity).WillByDefault(ReturnPointee(&imuGz)); \
    ON_CALL(turretSubsystem, yawLimited).WillByDefault(ReturnPointee(&yawLimited));

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController_chassis_not_moving_setpoint_actual_identical_0_out)
{
    SETUP_YAW_TEST();

    // Simulate chassis not moving, so turret world and chassis frame are identical

    turretSetpoint = 0;
    imuYaw = 0;
    imuGz = 0;
    currentYawValue.setValue(0);
    yawLimited = false;

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0));

    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController_chassis_not_moving_setpoint_gt_actual_output_positive)
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
    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController_chassis_not_moving_setpoint_lt_actual_output_negative)
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
    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController_chassis_rotated_positive_direction_world_setpoint_and_actual_equal_0_output)
{
    SETUP_YAW_TEST();

    // Simulate chassis moving with setpoint staying at 90 degrees

    // chassis frame yaw value 80, so chassis moved +10 degrees
    currentYawValue.setValue(80);
    // user input in world frame still equal to imu yaw, so output 0
    turretSetpoint = 90;
    imuYaw = 90;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0));

    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController_chassis_rotated_negative_direction_world_setpoint_and_actual_equal_0_output)
{
    SETUP_YAW_TEST();

    // Simulate chassis moving with setpoint staying at 90 degrees

    // yaw value 100, so chassis moved -10 degrees
    currentYawValue.setValue(100);
    turretSetpoint = 90;
    imuYaw = 90;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0));

    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController_chassis_rotated_positive_direction_world_setpoint_lt_actual_negative_output)
{
    SETUP_YAW_TEST();

    currentYawValue.setValue(90);
    turretSetpoint = 90;
    imuYaw = 100;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Lt(0)));

    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runYawPidController_chassis_rotated_negative_direction_world_setpoint_gt_actual_positive_output)
{
    SETUP_YAW_TEST();

    currentYawValue.setValue(90);
    turretSetpoint = 90;
    imuYaw = 80;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Gt(0)));

    turretController.runController(1, turretSetpoint);
}

#define SETUP_PITCH_TEST()                                                                      \
    Drivers drivers;                                                                            \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers);                                    \
    WorldFramePitchTurretImuCascadePidTurretController turretController(                        \
        &drivers,                                                                               \
        &turretSubsystem,                                                                       \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0},                                                         \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0});                                                        \
    float turretSetpoint = 0;                                                                   \
    ContiguousFloat worldFramePitchSetpoint(0, 0, 360);                                         \
    ContiguousFloat currentPitchValue(0, 0, 360);                                               \
    float imuPitch = 0;                                                                         \
    float imuGx = 0;                                                                            \
    ON_CALL(turretSubsystem, getCurrentPitchValue).WillByDefault(ReturnRef(currentPitchValue)); \
    ON_CALL(turretSubsystem, getPitchAngleFromCenter).WillByDefault(Return(0));                 \
    ON_CALL(drivers.turretMCBCanComm, getPitch).WillByDefault(ReturnPointee(&imuPitch));        \
    ON_CALL(drivers.turretMCBCanComm, getPitchVelocity).WillByDefault(ReturnPointee(&imuGx));

static inline float transformChassisFrameToWorldFrame(
    const float turretChassisFrameCurrAngle,
    const float turretWorldFrameCurrAngle,
    const float angleToTransform)
{
    return turretWorldFrameCurrAngle + (angleToTransform - turretChassisFrameCurrAngle);
}

static inline float transformWorldFrameValueToChassisFrame(
    const float turretChassisFrameCurrAngle,
    const float turretWorldFrameCurrAngle,
    const float angleToTransform)
{
    return turretChassisFrameCurrAngle + (angleToTransform - turretWorldFrameCurrAngle);
}

static float computeCGOffset(float pitchAngleFromCenter = 0.0f)
{
    return computeGravitationalForceOffset(
        TurretSubsystem::TURRET_CG_X,
        TurretSubsystem::TURRET_CG_Z,
        pitchAngleFromCenter,
        TurretSubsystem::GRAVITY_COMPENSATION_SCALAR);
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController_chassis_stationary_world_frame_stationary_output_0)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(computeCGOffset()));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Lt(computeCGOffset())));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Gt(computeCGOffset())));

    imuPitch = 90;
    turretSetpoint = 90;
    currentPitchValue.setValue(90);
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(Return(transformWorldFrameValueToChassisFrame(
            currentPitchValue.getValue(),
            imuPitch,
            turretSetpoint)));
    turretController.runController(1, turretSetpoint);

    // Simulate moving turret, setpoint stays the same as does turret chassis base imu

    currentPitchValue.setValue(100);
    turretController.runController(1, turretSetpoint);

    currentPitchValue.setValue(80);
    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController_chassis_tilted_up_world_setpoint_gt_actual_output_positive)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Gt(computeCGOffset())));

    turretSetpoint = 110;
    currentPitchValue.setValue(90);
    imuPitch = 90;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(Return(transformWorldFrameValueToChassisFrame(
            currentPitchValue.getValue(),
            imuPitch,
            turretSetpoint)));

    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController_chassis_tilted_up_world_setpoint_lt_actual_output_negative)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Lt(computeCGOffset())));

    turretSetpoint = 80;
    currentPitchValue.setValue(90);
    imuPitch = 90;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(Return(transformWorldFrameValueToChassisFrame(
            currentPitchValue.getValue(),
            imuPitch,
            turretSetpoint)));

    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController_chassis_tilted_down_world_setpoint_gt_actual_output_positive)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Gt(computeCGOffset())));

    turretSetpoint = 90;
    currentPitchValue.setValue(90);
    imuPitch = -10;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(Return(transformWorldFrameValueToChassisFrame(
            currentPitchValue.getValue(),
            imuPitch,
            turretSetpoint)));

    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController_chassis_tilted_down_world_setpoint_lt_actual_output_negative)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Lt(computeCGOffset())));

    turretSetpoint = 70;
    currentPitchValue.setValue(90);
    imuPitch = 90;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(Return(transformWorldFrameValueToChassisFrame(
            currentPitchValue.getValue(),
            imuPitch,
            turretSetpoint)));

    turretController.runController(1, turretSetpoint);
}

TEST(
    WorldFrameTurretImuTurretController,
    runPitchPidController_chassis_moved_world_frame_stationary_output_0)
{
    SETUP_PITCH_TEST();

    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(computeCGOffset())).Times(2);

    turretSetpoint = 90;
    currentPitchValue.setValue(80);
    imuPitch = 90;
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(Return(transformWorldFrameValueToChassisFrame(
            currentPitchValue.getValue(),
            imuPitch,
            turretSetpoint)));
    turretController.runController(1, turretSetpoint);

    imuPitch = 90;
    currentPitchValue.setValue(100);
    ON_CALL(turretSubsystem, getPitchSetpoint)
        .WillByDefault(Return(transformWorldFrameValueToChassisFrame(
            currentPitchValue.getValue(),
            imuPitch,
            turretSetpoint)));
    turretController.runController(1, turretSetpoint);
}
