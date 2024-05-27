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
#include "tap/mock/motor_interface_mock.hpp"

#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace testing;
using namespace aruwsrc::control::turret;
using namespace tap::algorithms;
using namespace aruwsrc::control::turret::algorithms;

class WorldFrameChassisImuTurretControllerTest : public Test
{
protected:
    static constexpr aruwsrc::control::turret::TurretMotorConfig TURRET_MOTOR_CONFIG = {
        .startAngle = M_PI_2,
        .startEncoderValue = 0,
        .minAngle = 0,
        .maxAngle = M_PI,
        .limitMotorAngles = false,
    };

    WorldFrameChassisImuTurretControllerTest()
        : turretMotor(&motor, TURRET_MOTOR_CONFIG),
          turretController(drivers, turretMotor, {1, 0, 0, 0, 1, 1, 0, 1, 0, 0}),
          chassisFrameMeasured(Angle(0)),
          chassisFrameSetpoint(Angle(0))
    {
    }

    void SetUp() override
    {
        ON_CALL(drivers.mpu6500, getYaw).WillByDefault(ReturnPointee(&mpu6500Yaw));
        ON_CALL(turretMotor, getChassisFrameMeasuredAngle)
            .WillByDefault(ReturnPointee(&chassisFrameMeasured));
        ON_CALL(turretMotor, getChassisFrameSetpoint)
            .WillByDefault(ReturnPointee(&chassisFrameSetpoint));
        ON_CALL(turretMotor, getConfig).WillByDefault(ReturnRef(TURRET_MOTOR_CONFIG));
    }

    tap::Drivers drivers;
    testing::NiceMock<tap::mock::MotorInterfaceMock> motor;
    testing::NiceMock<aruwsrc::mock::TurretMotorMock> turretMotor;
    WorldFrameYawChassisImuTurretController turretController;
    float mpu6500Yaw = 0;
    WrappedFloat chassisFrameMeasured;
    WrappedFloat chassisFrameSetpoint;
};

TEST_F(
    WorldFrameChassisImuTurretControllerTest,
    runYawPidController_setpoint_actual_identical_output_0)
{
    chassisFrameMeasured = Angle(M_PI_2);

    EXPECT_CALL(turretMotor, setMotorOutput(FloatNear(0, 1E-3)));

    turretController.runController(1, Angle(M_PI_2));
}

TEST_F(
    WorldFrameChassisImuTurretControllerTest,
    runYawPidController_setpoint_gt_actual_output_positive)
{
    chassisFrameMeasured = Angle(modm::toRadian(80));

    EXPECT_CALL(turretMotor, setMotorOutput(Gt(0)));

    turretController.runController(1, Angle(M_PI_2));
}

TEST_F(
    WorldFrameChassisImuTurretControllerTest,
    runYawPidController_setpoint_lt_actual_output_negative)
{
    chassisFrameMeasured = Angle(modm::toRadian(100));

    EXPECT_CALL(turretMotor, setMotorOutput(Lt(0)));

    turretController.runController(1, Angle(M_PI_2));
}

TEST_F(
    WorldFrameChassisImuTurretControllerTest,
    runYawPidController_chassis_frame_rotated_setpoint_actual_equal_0_output)
{
    chassisFrameMeasured = Angle(0);
    mpu6500Yaw = 90;

    EXPECT_CALL(turretMotor, setMotorOutput(FloatNear(0, 1E-3)));

    turretController.runController(1, Angle(M_PI_2));
}
