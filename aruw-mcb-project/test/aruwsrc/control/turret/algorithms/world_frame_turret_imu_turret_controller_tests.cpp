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
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
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
          worldFrameSetpoint(0, 0, M_TWOPI),
          currentValue(0, 0, M_TWOPI)
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
              &turretSubsystem.yawMotor,
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0},
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0})
    {
    }

    void SetUp() override
    {
        ON_CALL(turretSubsystem.yawMotor, getChassisFrameMeasuredAngle)
            .WillByDefault(ReturnRef(currentValue));
        ON_CALL(drivers.turretMCBCanComm, getYaw).WillByDefault(ReturnPointee(&imuValue));
        ON_CALL(drivers.turretMCBCanComm, getYawVelocity)
            .WillByDefault(ReturnPointee(&imuVelocity));
        yawMotorConfig.limitMotorAngles = false;
        ON_CALL(turretSubsystem.yawMotor, getConfig).WillByDefault(ReturnRef(yawMotorConfig));
    }

    WorldFrameYawTurretImuCascadePidTurretController turretController;
    TurretMotorConfig yawMotorConfig;
};

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_not_moving_setpoint_actual_identical_0_out)
{
    EXPECT_CALL(turretSubsystem.yawMotor, setMotorOutput(0));

    turretController.runController(1, 0);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_not_moving_setpoint_gt_actual_output_positive)
{
    // User input > current angle, output should be positive
    currentValue.setValue(modm::toRadian(modm::toRadian(80)));
    imuValue = modm::toRadian(modm::toRadian(80));

    EXPECT_CALL(turretSubsystem.yawMotor, setMotorOutput(Gt(0)));

    turretController.runController(1, M_PI_2);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_not_moving_setpoint_lt_actual_output_negative)
{
    // Setpoint < current angle, output should be negative
    currentValue.setValue(modm::toRadian(110));
    imuValue = modm::toRadian(110);

    EXPECT_CALL(turretSubsystem.yawMotor, setMotorOutput(Lt(0)));

    turretController.runController(1, modm::toRadian(modm::toRadian(100)));
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_positive_direction_world_setpoint_and_actual_equal_0_output)
{
    // chassis frame yaw value modm::toRadian(80), so chassis moved +10 degrees
    currentValue.setValue(modm::toRadian(modm::toRadian(80)));
    // user input in world frame still equal to imu yaw, so output 0
    imuValue = M_PI_2;

    EXPECT_CALL(turretSubsystem.yawMotor, setMotorOutput(0));

    turretController.runController(1, M_PI_2);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_negative_direction_world_setpoint_and_actual_equal_0_output)
{
    // yaw value modm::toRadian(100), so chassis moved -10 degrees
    currentValue.setValue(modm::toRadian(100));
    imuValue = M_PI_2;

    EXPECT_CALL(turretSubsystem.yawMotor, setMotorOutput(0));

    turretController.runController(1, M_PI_2);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_positive_direction_world_setpoint_lt_actual_negative_output)
{
    currentValue.setValue(M_PI_2);
    imuValue = modm::toRadian(100);

    EXPECT_CALL(turretSubsystem.yawMotor, setMotorOutput(Lt(0)));

    turretController.runController(1, M_PI_2);
}

TEST_F(
    YawWorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_negative_direction_world_setpoint_gt_actual_positive_output)
{
    currentValue.setValue(M_PI_2);
    imuValue = modm::toRadian(80);
    EXPECT_CALL(turretSubsystem.yawMotor, setMotorOutput(Gt(0)));

    turretController.runController(1, M_PI_2);
}

#define SETUP_PITCH_TEST()                                   \
    Drivers drivers;                                         \
    NiceMock<TurretSubsystemMock> turretSubsystem(&drivers); \
    ;                                                        \
    float turretSetpoint = 0;                                \
    ContiguousFloat worldFramePitchSetpoint(0, 0, M_TWOPI);  \
    ContiguousFloat currentValue(0, 0, M_TWOPI);             \
    float imuValue = 0;                                      \
    float imuVelocity = 0;

class PitchWorldFrameTurretImuTurretControllerTest : public WorldFrameTurretImuTurretControllerTest
{
protected:
    PitchWorldFrameTurretImuTurretControllerTest()
        : turretController(
              &drivers,
              &turretSubsystem.pitchMotor,
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0},
              {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}),
          chassisFrameSetpoint(0)
    {
    }

    void SetUp() override
    {
        ON_CALL(turretSubsystem.pitchMotor, getChassisFrameMeasuredAngle)
            .WillByDefault(ReturnRef(currentValue));
        ON_CALL(turretSubsystem.pitchMotor, getAngleFromCenter).WillByDefault(Return(0));
        ON_CALL(drivers.turretMCBCanComm, getPitch).WillByDefault(ReturnPointee(&imuValue));
        ON_CALL(drivers.turretMCBCanComm, getPitchVelocity)
            .WillByDefault(ReturnPointee(&imuVelocity));

        ON_CALL(turretSubsystem.pitchMotor, setChassisFrameSetpoint)
            .WillByDefault([&](float setpoint) { chassisFrameSetpoint = setpoint; });
        ON_CALL(turretSubsystem.pitchMotor, getChassisFrameSetpoint)
            .WillByDefault(ReturnPointee(&chassisFrameSetpoint));
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
    float chassisFrameSetpoint;
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
    EXPECT_CALL(turretSubsystem.pitchMotor, setMotorOutput(computeCGOffset()));
    // EXPECT_CALL(turretSubsystem.pitchMotor, setMotorOutput(computeCGOffset()));
    // EXPECT_CALL(turretSubsystem.pitchMotor, setMotorOutput(computeCGOffset()));

    turretSetpoint = M_PI_2;
    imuValue = M_PI_2;

    currentValue.setValue(M_PI_2);
    turretController.runController(1, turretSetpoint);

    // currentValue.setValue(modm::toRadian(100));
    // turretController.runController(1, turretSetpoint);

    // currentValue.setValue(modm::toRadian(80));
    // turretController.runController(1, turretSetpoint);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_tilted_up_world_setpoint_gt_actual_output_positive)
{
    EXPECT_CALL(turretSubsystem.pitchMotor, setMotorOutput(Gt(computeCGOffset())));

    turretSetpoint = modm::toRadian(110);
    currentValue.setValue(M_PI_2);
    imuValue = M_PI_2;

    turretController.runController(1, turretSetpoint);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_tilted_up_world_setpoint_lt_actual_output_negative)
{
    EXPECT_CALL(turretSubsystem.pitchMotor, setMotorOutput(Lt(computeCGOffset())));

    turretSetpoint = modm::toRadian(80);
    currentValue.setValue(M_PI_2);
    imuValue = M_PI_2;

    turretController.runController(1, turretSetpoint);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_tilted_down_world_setpoint_gt_actual_output_positive)
{
    EXPECT_CALL(turretSubsystem.pitchMotor, setMotorOutput(Gt(computeCGOffset())));

    turretSetpoint = M_PI_2;
    currentValue.setValue(M_PI_2);
    imuValue = modm::toRadian(-10);

    turretController.runController(1, turretSetpoint);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_tilted_down_world_setpoint_lt_actual_output_negative)
{
    EXPECT_CALL(turretSubsystem.pitchMotor, setMotorOutput(Lt(computeCGOffset())));

    turretSetpoint = modm::toRadian(70);
    currentValue.setValue(M_PI_2);
    imuValue = M_PI_2;

    turretController.runController(1, turretSetpoint);
}

TEST_F(
    PitchWorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_moved_world_frame_stationary_output_0)
{
    EXPECT_CALL(turretSubsystem.pitchMotor, setMotorOutput(computeCGOffset())).Times(2);

    turretSetpoint = M_PI_2;
    currentValue.setValue(modm::toRadian(modm::toRadian(80)));
    imuValue = M_PI_2;
    turretController.runController(1, turretSetpoint);

    imuValue = M_PI_2;
    currentValue.setValue(modm::toRadian(100));
    turretController.runController(1, turretSetpoint);
}
