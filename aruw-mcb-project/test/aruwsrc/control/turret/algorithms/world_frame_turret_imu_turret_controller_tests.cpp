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
#include "tap/mock/dji_motor_mock.hpp"

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"
#include "aruwsrc/control/turret/algorithms/world_frame_turret_imu_turret_controller.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"
#include "aruwsrc/mock/turret_mcb_can_comm_mock.hpp"
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
        : posPid({100, 0, 0, 0, 100, 1, 0, 1, 0, 0}),
          velPid({100, 0, 0, 0, 100, 1, 0, 1, 0, 0}),
          motorConfig{.limitMotorAngles = false},
          djiMotor(&drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "motor"),
          turretMCBCanCommBus1(&drivers, tap::can::CanBus::CAN_BUS1)
    {
    }

    void SetUp() override
    {
        ON_CALL(djiMotor, isMotorOnline).WillByDefault(Return(true));
        ON_CALL(djiMotor, getEncoderUnwrapped)
            .WillByDefault(
                [&]() {
                    return chassisFrameUnwrappedMeasurement * tap::motor::DjiMotor::ENC_RESOLUTION /
                           M_TWOPI;
                });
        ON_CALL(djiMotor, setDesiredOutput)
            .WillByDefault([&](int32_t desiredOutput)
                           { return djiMotor.DjiMotor::setDesiredOutput(desiredOutput); });
        ON_CALL(djiMotor, getOutputDesired)
            .WillByDefault([&]() { return djiMotor.DjiMotor::getOutputDesired(); });

        ON_CALL(turretMCBCanCommBus1, getYawUnwrapped)
            .WillByDefault(ReturnPointee(&turretFrameImuValue));
        ON_CALL(turretMCBCanCommBus1, getYawVelocity)
            .WillByDefault(ReturnPointee(&turretFrameImuVelocity));

        ON_CALL(turretMCBCanCommBus1, getPitchUnwrapped)
            .WillByDefault(ReturnPointee(&turretFrameImuValue));
        ON_CALL(turretMCBCanCommBus1, getPitchVelocity)
            .WillByDefault(ReturnPointee(&turretFrameImuVelocity));
    }

    void setDefaultMotorBehavior(NiceMock<TurretMotorMock> &turretMotor)
    {
        ON_CALL(turretMotor, getChassisFrameUnwrappedMeasuredAngle)
            .WillByDefault(ReturnPointee(&chassisFrameUnwrappedMeasurement));

        ON_CALL(turretMotor, getConfig).WillByDefault(ReturnRef(motorConfig));

        ON_CALL(turretMotor, setChassisFrameSetpoint)
            .WillByDefault([&](float setpoint)
                           { turretMotor.TurretMotor::setChassisFrameSetpoint(setpoint); });

        ON_CALL(turretMotor, getChassisFrameSetpoint)
            .WillByDefault([&]() { return turretMotor.TurretMotor::getChassisFrameSetpoint(); });
    }

    tap::Drivers drivers;
    tap::algorithms::SmoothPid posPid;
    tap::algorithms::SmoothPid velPid;
    float chassisFrameUnwrappedMeasurement = 0;
    float turretFrameImuValue = 0;
    float turretFrameImuVelocity = 0;
    TurretMotorConfig motorConfig;
    NiceMock<tap::mock::DjiMotorMock> djiMotor;
    NiceMock<aruwsrc::mock::TurretMCBCanCommMock> turretMCBCanCommBus1;
};

TEST_F(WorldFrameTurretImuTurretControllerTest, runYawPidController_world_frame_setpoint_limited)
{
    motorConfig = {
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = -M_TWOPI,
        .maxAngle = M_TWOPI,
        .limitMotorAngles = true,
    };

    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFrameYawTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    turretController.runController(1, Angle(-2.0f * M_TWOPI));

    EXPECT_NEAR(0, turretController.getSetpoint().minDifference(-M_TWOPI), 1e-5f);

    turretController.runController(1, Angle(2.0f * M_TWOPI));

    EXPECT_NEAR(0, turretController.getSetpoint().minDifference(M_TWOPI), 1e-5f);

    turretFrameImuValue = 2.0f * M_TWOPI;

    turretController.runController(1, Angle(0));

    EXPECT_NEAR(0, turretController.getSetpoint().minDifference(M_TWOPI), 1e-5f);

    turretFrameImuValue = 0;
    chassisFrameUnwrappedMeasurement = 2.0f * M_TWOPI;
    turretMotor.updateMotorAngle();

    turretController.runController(1, Angle(0));

    EXPECT_NEAR(0, turretController.getSetpoint().minDifference(-M_TWOPI), 1e-5f);
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_not_moving_setpoint_actual_identical_0_out)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFrameYawTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    turretController.runController(1, Angle(0));

    EXPECT_EQ(0, turretMotor.getMotorOutput());
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_not_moving_setpoint_gt_actual_output_positive)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFrameYawTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    // User input > current angle, output should be positive
    chassisFrameUnwrappedMeasurement = modm::toRadian(80);
    turretMotor.updateMotorAngle();
    turretFrameImuValue = modm::toRadian(80);

    turretController.runController(1, Angle(M_PI_2));

    EXPECT_GT(turretMotor.getMotorOutput(), 0);
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_not_moving_setpoint_lt_actual_output_negative)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFrameYawTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    // Setpoint < current angle, output should be negative
    chassisFrameUnwrappedMeasurement = modm::toRadian(110);
    turretMotor.updateMotorAngle();
    turretFrameImuValue = modm::toRadian(110);

    turretController.runController(1, Angle(modm::toRadian(modm::toRadian(100))));

    EXPECT_LT(turretMotor.getMotorOutput(), 0);
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_positive_direction_world_setpoint_and_actual_equal_0_output)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFrameYawTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    // chassis frame yaw value modm::toRadian(80), so chassis moved +10 degrees
    chassisFrameUnwrappedMeasurement = modm::toRadian(80);
    turretMotor.updateMotorAngle();
    // user input in world frame still equal to imu yaw, so output 0
    turretFrameImuValue = M_PI_2;

    turretController.runController(1, Angle(M_PI_2));

    EXPECT_EQ(turretMotor.getMotorOutput(), 0);
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_negative_direction_world_setpoint_and_actual_equal_0_output)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFrameYawTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    // yaw value modm::toRadian(100), so chassis moved -10 degrees
    chassisFrameUnwrappedMeasurement = modm::toRadian(100);
    turretMotor.updateMotorAngle();
    turretFrameImuValue = M_PI_2;

    turretController.runController(1, Angle(M_PI_2));

    EXPECT_EQ(turretMotor.getMotorOutput(), 0);
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_positive_direction_world_setpoint_lt_actual_negative_output)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFrameYawTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    chassisFrameUnwrappedMeasurement = M_PI_2;
    turretMotor.updateMotorAngle();
    turretFrameImuValue = modm::toRadian(100);

    turretController.runController(1, Angle(M_PI_2));

    EXPECT_LT(turretMotor.getMotorOutput(), 0);
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runYawPidController_chassis_rotated_negative_direction_world_setpoint_gt_actual_positive_output)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFrameYawTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    chassisFrameUnwrappedMeasurement = M_PI_2;
    turretMotor.updateMotorAngle();
    turretFrameImuValue = modm::toRadian(80);

    turretController.runController(1, Angle(M_PI_2));

    EXPECT_GT(turretMotor.getMotorOutput(), 0);
}

// Pitch controller tests

static int16_t computeCGOffset(float pitchAngleFromCenter)
{
    return computeGravitationalForceOffset(
        TURRET_CG_X,
        TURRET_CG_Z,
        -pitchAngleFromCenter,
        GRAVITY_COMPENSATION_SCALAR);
}

TEST_F(WorldFrameTurretImuTurretControllerTest, runPitchPidController_world_frame_setpoint_limited)
{
    motorConfig = {
        .startAngle = 0,
        .startEncoderValue = 0,
        .minAngle = -M_TWOPI,
        .maxAngle = M_TWOPI,
        .limitMotorAngles = true,
    };

    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFramePitchTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    turretController.runController(1, Angle(-2.0f * M_TWOPI));

    EXPECT_NEAR(0, turretController.getSetpoint().minDifference(-M_TWOPI), 1e-5f);

    turretController.runController(1, Angle(2.0f * M_TWOPI));

    EXPECT_NEAR(0, turretController.getSetpoint().minDifference(M_TWOPI), 1e-5f);

    turretFrameImuValue = 2.0f * M_TWOPI;

    turretController.runController(1, Angle(0));

    EXPECT_NEAR(0, turretController.getSetpoint().minDifference(M_TWOPI), 1e-5f);

    turretFrameImuValue = 0;
    chassisFrameUnwrappedMeasurement = 2.0f * M_TWOPI;
    turretMotor.updateMotorAngle();

    turretController.runController(1, Angle(0));

    EXPECT_NEAR(0, turretController.getSetpoint().minDifference(-M_TWOPI), 1e-5f);
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_not_moving_setpoint_actual_identical_0_out)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFramePitchTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    turretController.runController(1, Angle(0));

    EXPECT_EQ(computeCGOffset(turretMotor.getAngleFromCenter()), turretMotor.getMotorOutput());
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_not_moving_setpoint_gt_actual_output_positive)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFramePitchTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    // User input > current angle, output should be positive
    chassisFrameUnwrappedMeasurement = modm::toRadian(80);
    turretMotor.updateMotorAngle();
    turretFrameImuValue = modm::toRadian(80);

    turretController.runController(1, Angle(M_PI_2));

    EXPECT_GT(turretMotor.getMotorOutput(), computeCGOffset(turretMotor.getAngleFromCenter()));
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_not_moving_setpoint_lt_actual_output_negative)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFramePitchTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    // Setpoint < current angle, output should be negative
    chassisFrameUnwrappedMeasurement = modm::toRadian(110);
    turretMotor.updateMotorAngle();
    turretFrameImuValue = modm::toRadian(110);

    turretController.runController(1, Angle(modm::toRadian(modm::toRadian(100))));

    EXPECT_LT(turretMotor.getMotorOutput(), computeCGOffset(turretMotor.getAngleFromCenter()));
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_rotated_positive_direction_world_setpoint_and_actual_equal_0_output)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFramePitchTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    // chassis frame yaw value modm::toRadian(80), so chassis moved +10 degrees
    chassisFrameUnwrappedMeasurement = modm::toRadian(80);
    turretMotor.updateMotorAngle();
    // user input in world frame still equal to imu yaw, so output 0
    turretFrameImuValue = M_PI_2;

    turretController.runController(1, Angle(M_PI_2));

    EXPECT_EQ(turretMotor.getMotorOutput(), computeCGOffset(turretMotor.getAngleFromCenter()));
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_rotated_negative_direction_world_setpoint_and_actual_equal_0_output)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFramePitchTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    // yaw value modm::toRadian(100), so chassis moved -10 degrees
    chassisFrameUnwrappedMeasurement = modm::toRadian(100);
    turretMotor.updateMotorAngle();
    turretFrameImuValue = M_PI_2;

    turretController.runController(1, Angle(M_PI_2));

    EXPECT_EQ(turretMotor.getMotorOutput(), computeCGOffset(turretMotor.getAngleFromCenter()));
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_rotated_positive_direction_world_setpoint_lt_actual_negative_output)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFramePitchTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    chassisFrameUnwrappedMeasurement = M_PI_2;
    turretMotor.updateMotorAngle();
    turretFrameImuValue = modm::toRadian(100);

    turretController.runController(1, Angle(M_PI_2));

    EXPECT_LT(turretMotor.getMotorOutput(), computeCGOffset(turretMotor.getAngleFromCenter()));
}

TEST_F(
    WorldFrameTurretImuTurretControllerTest,
    runPitchPidController_chassis_rotated_negative_direction_world_setpoint_gt_actual_positive_output)
{
    TurretMotor turretMotor(&djiMotor, motorConfig);
    turretMotor.updateMotorAngle();

    WorldFramePitchTurretImuCascadePidTurretController turretController(
        turretMCBCanCommBus1,
        turretMotor,
        posPid,
        velPid);

    chassisFrameUnwrappedMeasurement = M_PI_2;
    turretMotor.updateMotorAngle();
    turretFrameImuValue = modm::toRadian(80);

    turretController.runController(1, Angle(M_PI_2));

    EXPECT_GT(turretMotor.getMotorOutput(), computeCGOffset(turretMotor.getAngleFromCenter()));
}
