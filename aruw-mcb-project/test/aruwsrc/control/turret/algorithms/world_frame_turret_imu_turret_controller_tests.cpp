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
#include "aruwsrc/control/turret/turret_controller_constants.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace aruwsrc;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::turret::algorithms;
using namespace aruwsrc::mock;
using namespace testing;

class WorldFrameTurretImuTurretControllerTest : public Test
{
protected:
    WorldFrameTurretImuTurretControllerTest()
        : turretSubsystem(&drivers),
          worldFrameSetpoint(0, 0, 360),
          currentValue(0, 0, 360)
    {
    }

    Drivers drivers;
    NiceMock<TurretSubsystemMock> turretSubsystem;
    ContiguousFloat worldFrameSetpoint;
    ContiguousFloat currentValue;
    float imuValue = 0;
    float imuVelocity = 0;
};

class YawWorldFrameTurretImuTurretControllerTest : public WorldFrameTurretImuTurretControllerTest
{
protected:
    YawWorldFrameTurretImuTurretControllerTest()
        : turretController(
              &drivers,
              &turretSubsystem,
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0},
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0})
    {
    }

    void SetUp() override
    {
        ON_CALL(turretSubsystem, getCurrentYawValue).WillByDefault(ReturnRef(currentValue));
        ON_CALL(drivers.turretMCBCanComm, getYaw).WillByDefault(ReturnPointee(&imuValue));
        ON_CALL(drivers.turretMCBCanComm, getYawVelocity)
            .WillByDefault(ReturnPointee(&imuVelocity));
        ON_CALL(turretSubsystem, yawLimited).WillByDefault(ReturnPointee(&yawLimited));
    }

    bool yawLimited = false;
    WorldFrameYawTurretImuCascadePidTurretController turretController;
};

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_not_moving_setpoint_actual_identical_0_out)
{
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0));

    turretController.runController(1, 0);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_not_moving_setpoint_gt_actual_output_positive)
{
    // User input > current angle, output should be positive
    currentValue.setValue(80);
    imuValue = 80;

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Gt(0)));

    turretController.runController(1, 90);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_not_moving_setpoint_lt_actual_output_negative)
{
    // Setpoint < current angle, output should be negative
    currentValue.setValue(110);
    imuValue = 110;

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Lt(0)));

    turretController.runController(1, 100);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_positive_direction_world_setpoint_and_actual_equal_0_output)
{
    // chassis frame yaw value 80, so chassis moved +10 degrees
    currentValue.setValue(80);
    // user input in world frame still equal to imu yaw, so output 0
    imuValue = 90;

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0));

    turretController.runController(1, 90);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_negative_direction_world_setpoint_and_actual_equal_0_output)
{
    // yaw value 100, so chassis moved -10 degrees
    currentValue.setValue(100);
    imuValue = 90;

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(0));

    turretController.runController(1, 90);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_positive_direction_world_setpoint_lt_actual_negative_output)
{
    currentValue.setValue(90);
    imuValue = 100;

    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Lt(0)));

    turretController.runController(1, 90);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_negative_direction_world_setpoint_gt_actual_positive_output)
{
    currentValue.setValue(90);
    imuValue = 80;
    EXPECT_CALL(turretSubsystem, setYawMotorOutput(Gt(0)));

    turretController.runController(1, 90);
}

#define SETUP_PITCH_TEST()                                   \
    Drivers drivers;                                         \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers); \
    ;                                                        \
    float turretSetpoint = 0;                                \
    ContiguousFloat worldFramePitchSetpoint(0, 0, 360);      \
    ContiguousFloat currentValue(0, 0, 360);                 \
    float imuValue = 0;                                      \
    float imuVelocity = 0;

class PitchWorldFrameTurretImuTurretControllerTest : public WorldFrameTurretImuTurretControllerTest
{
protected:
    PitchWorldFrameTurretImuTurretControllerTest()
        : turretController(
              &drivers,
              &turretSubsystem,
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0},
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0})
    {
    }

    void SetUp() override
    {
        ON_CALL(turretSubsystem, getCurrentPitchValue).WillByDefault(ReturnRef(currentValue));
        ON_CALL(turretSubsystem, getPitchAngleFromCenter).WillByDefault(Return(0));
        ON_CALL(drivers.turretMCBCanComm, getPitch).WillByDefault(ReturnPointee(&imuValue));
        ON_CALL(drivers.turretMCBCanComm, getPitchVelocity)
            .WillByDefault(ReturnPointee(&imuVelocity));

        ON_CALL(turretSubsystem, setPitchSetpoint).WillByDefault([&](float setpiont) {
            chassisFrameSetpoint = setpiont;
        });
        ON_CALL(turretSubsystem, getPitchSetpoint).WillByDefault([&]() {
            return chassisFrameSetpoint;
        });
    }

    static inline float transformWorldFrameValueToChassisFrame(
        const float turretChassisFrameCurrAngle,
        const float turretWorldFrameCurrAngle,
        const float angleToTransform)
    {
        return turretChassisFrameCurrAngle + (angleToTransform - turretWorldFrameCurrAngle);
    }

    float getPitchSetpoint()
    {
        return transformWorldFrameValueToChassisFrame(
            currentValue.getValue(),
            imuValue,
            turretSetpoint);
    }

    float turretSetpoint = 0;
    bool yawLimited = false;
    WorldFramePitchTurretImuCascadePidTurretController turretController;

private:
    float chassisFrameSetpoint = 0;
};

static float computeCGOffset(float pitchAngleFromCenter = 0.0f)
{
    return computeGravitationalForceOffset(
        TURRET_CG_X,
        TURRET_CG_Z,
        pitchAngleFromCenter,
        GRAVITY_COMPENSATION_SCALAR);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_moving_world_frame_stationary_output_0)
{
    InSequence seq;
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(computeCGOffset()));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(computeCGOffset()));
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(computeCGOffset()));

    turretSetpoint = 90;
    imuValue = 90;

    currentValue.setValue(90);
    turretController.runController(1, turretSetpoint);

    currentValue.setValue(100);
    turretController.runController(1, turretSetpoint);

    currentValue.setValue(80);
    turretController.runController(1, turretSetpoint);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_tilted_up_world_setpoint_gt_actual_output_positive)
{
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Gt(computeCGOffset())));

    turretSetpoint = 110;
    currentValue.setValue(90);
    imuValue = 90;

    turretController.runController(1, turretSetpoint);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_tilted_up_world_setpoint_lt_actual_output_negative)
{
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Lt(computeCGOffset())));

    turretSetpoint = 80;
    currentValue.setValue(90);
    imuValue = 90;

    turretController.runController(1, turretSetpoint);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_tilted_down_world_setpoint_gt_actual_output_positive)
{
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Gt(computeCGOffset())));

    turretSetpoint = 90;
    currentValue.setValue(90);
    imuValue = -10;

    turretController.runController(1, turretSetpoint);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_tilted_down_world_setpoint_lt_actual_output_negative)
{
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(Lt(computeCGOffset())));

    turretSetpoint = 70;
    currentValue.setValue(90);
    imuValue = 90;

    turretController.runController(1, turretSetpoint);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_moved_world_frame_stationary_output_0)
{
    EXPECT_CALL(turretSubsystem, setPitchMotorOutput(computeCGOffset())).Times(2);

    turretSetpoint = 90;
    currentValue.setValue(80);
    imuValue = 90;
    turretController.runController(1, turretSetpoint);

    imuValue = 90;
    currentValue.setValue(100);
    turretController.runController(1, turretSetpoint);
}
