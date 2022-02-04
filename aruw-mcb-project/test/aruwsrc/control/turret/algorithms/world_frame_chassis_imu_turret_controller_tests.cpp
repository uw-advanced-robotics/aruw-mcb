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

#include "aruwsrc/control/turret/algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace testing;

#define SETUP_SINGLE_PID_TEST()                                             \
    aruwsrc::Drivers drivers;                                               \
    tap::algorithms::ContiguousFloat worldFrameYawSetpoint(0, 0, 360);      \
    testing::NiceMock<aruwsrc::mock::TurretSubsystemMock> turret(&drivers); \
    WorldFrameYawChassisImuTurretController turretController(               \
        &drivers,                                                           \
        &turret,                                                            \
        {1, 0, 0, 0, 1, 1, 0, 1, 0, 0});

using namespace aruwsrc::control::turret;
using namespace aruwsrc::control::turret::algorithms;

TEST(WorldFrameChassisImuTurretController, runYawPidController_setpoint_actual_identical_output_0)
{
    // Setpoint used in gmock pointee
    float yawSetpoint = 0.0f;
    // Value used in gmock return reference
    tap::algorithms::ContiguousFloat yawValue(0, 0, 360);

    SETUP_SINGLE_PID_TEST();

    yawSetpoint = 90.0f;
    yawValue.setValue(90);
    worldFrameYawSetpoint.setValue(yawSetpoint);
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(yawValue));
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(Return(0));

    EXPECT_CALL(turret, setYawMotorOutput(FloatNear(0, 1E-3)));

    turretController.runController(1, yawSetpoint);
}

TEST(WorldFrameChassisImuTurretController, runYawPidController_setpoint_gt_actual_output_positive)
{
    // Setpoint used in gmock pointee
    float yawSetpoint = 0.0f;
    // Value used in gmock return reference
    tap::algorithms::ContiguousFloat yawValue(0, 0, 360);
    SETUP_SINGLE_PID_TEST();

    // setpoint > value, output should be > 0
    yawSetpoint = 90;
    yawValue.setValue(80);
    worldFrameYawSetpoint.setValue(yawSetpoint);
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(yawValue));
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(Return(0));

    EXPECT_CALL(turret, setYawMotorOutput(Gt(0)));

    turretController.runController(1, yawSetpoint);
}

TEST(WorldFrameChassisImuTurretController, runYawPidController_setpoint_lt_actual_output_negative)
{
    // Setpoint used in gmock pointee
    float yawSetpoint = 0.0f;
    // Value used in gmock return reference
    tap::algorithms::ContiguousFloat yawValue(0, 0, 360);

    SETUP_SINGLE_PID_TEST();

    // setpoint < value, output should be < 0
    yawSetpoint = 90;
    yawValue.setValue(100);
    worldFrameYawSetpoint.setValue(yawSetpoint);
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(yawValue));
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(Return(0));

    EXPECT_CALL(turret, setYawMotorOutput(Lt(0)));

    turretController.runController(1, yawSetpoint);
}

TEST(
    WorldFrameChassisImuTurretController,
    runYawPidController_chassis_frame_rotated_setpoint_actual_equal_0_output)
{
    // Setpoint used in gmock pointee
    float yawSetpoint = 0.0f;
    // Value used in gmock return reference
    tap::algorithms::ContiguousFloat yawValue(0, 0, 360);

    SETUP_SINGLE_PID_TEST();

    yawSetpoint = 90;
    yawValue.setValue(0);
    worldFrameYawSetpoint.setValue(yawSetpoint);
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(yawValue));
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(Return(90));

    EXPECT_CALL(turret, setYawMotorOutput(FloatNear(0, 1E-3)));

    turretController.runController(1, yawSetpoint);
}
