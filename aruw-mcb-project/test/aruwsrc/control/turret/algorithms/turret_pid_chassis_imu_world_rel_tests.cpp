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

#include "aruwsrc/control/turret/algorithms/turret_pid_chassis_imu_world_rel.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace testing;

#define SETUP_SINGLE_PID_TEST()                                             \
    tap::Drivers drivers;                                                   \
    tap::algorithms::ContiguousFloat worldFrameYawSetpoint(0, 0, 360);      \
    testing::NiceMock<aruwsrc::mock::TurretSubsystemMock> turret(&drivers); \
    tap::algorithms::SmoothPid positionPid(1, 0, 0, 0, 1, 1, 0, 1, 0);

#define RUN_SINGLE_PID_YAW_CONTROLLER()  \
    runSinglePidYawWorldFrameController( \
        1,                               \
        yawSetpoint,                     \
        0,                               \
        worldFrameYawSetpoint,           \
        &drivers,                        \
        &turret,                         \
        positionPid);

using namespace aruwsrc::control::turret::chassis_imu_world_rel;

TEST(
    TurretPidChassisImuWorldRelTests,
    runSinglePidYawWorldFrameController__not_limited_chassis_not_moving)
{
    // Setpoint used in gmock pointee
    float yawSetpoint = 0.0f;
    // Value used in gmock return reference
    tap::algorithms::ContiguousFloat yawValue(0, 0, 360);

    SETUP_SINGLE_PID_TEST()

    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(Return(0));
    ON_CALL(turret, yawLimited).WillByDefault(Return(false));

    // Set up yaw setpoint stuff
    ON_CALL(turret, getYawSetpoint).WillByDefault(ReturnPointee(&yawSetpoint));

    // Set up yaw current value stuff
    ON_CALL(turret, getCurrentYawValue).WillByDefault(ReturnRef(yawValue));
    ON_CALL(turret, getYawVelocity)
        .WillByDefault(Return(0));  // For the purpose of this test, ignore velocity

    // Check PID output for various inputs

    // Input/output same, 0 output
    yawSetpoint = 90.0f;
    yawValue.setValue(90);
    worldFrameYawSetpoint.setValue(yawSetpoint);

    RUN_SINGLE_PID_YAW_CONTROLLER();

    EXPECT_FLOAT_EQ(0.0f, positionPid.getOutput());

    // setpoint < value, output should be < 0
    positionPid.reset();
    yawSetpoint = 90;
    yawValue.setValue(100);
    worldFrameYawSetpoint.setValue(yawSetpoint);

    RUN_SINGLE_PID_YAW_CONTROLLER();

    EXPECT_GT(0.0f, positionPid.getOutput());

    // setpoint > value, output should be < 0
    positionPid.reset();
    yawSetpoint = 90;
    yawValue.setValue(80);
    worldFrameYawSetpoint.setValue(yawSetpoint);

    RUN_SINGLE_PID_YAW_CONTROLLER();

    EXPECT_LT(0.0f, positionPid.getOutput());

    // Move chassis orientation then re-check PID output. Now the yaw value will be
    // chassis relative, so 0 degrees in the chassis frame is the same as 90 in the
    // world frame if the chassis was rotated 90 degrees, for example

    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(Return(90));

    // setpoint == value, output 0
    positionPid.reset();
    yawSetpoint = 90;
    yawValue.setValue(0);
    worldFrameYawSetpoint.setValue(yawSetpoint);

    // Since the chassis is rotated 90 degrees relative to the start, we want the
    // turret to be pointed forward relative to the start, or 0 degrees relative
    // to the chassis.
    EXPECT_CALL(turret, setYawSetpoint(0));

    RUN_SINGLE_PID_YAW_CONTROLLER();

    EXPECT_FLOAT_EQ(0.0f, positionPid.getOutput());

    // Move chassis orientation again, if setpoint in chassis frame is the same as value
    // in chassis frame and the pid controller returns 0, we are good to go.

    // chassis rotated 30 degrees
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(Return(330));

    positionPid.reset();
    yawSetpoint = 90;
    yawValue.setValue(120);  // 90 (start) + 30 (rotated amnt), means in world frame value still 90
    worldFrameYawSetpoint.setValue(yawSetpoint);

    EXPECT_CALL(
        turret,
        setYawSetpoint(-240));  // wrapped between [0, 360), this value is identical to 120.

    RUN_SINGLE_PID_YAW_CONTROLLER();

    EXPECT_FLOAT_EQ(0.0f, positionPid.getOutput());
}
