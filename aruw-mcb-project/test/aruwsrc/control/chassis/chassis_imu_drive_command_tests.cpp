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

#include <vector>

#include <gtest/gtest.h>

#include "aruwsrc/control/chassis/chassis_imu_drive_command.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/mock/chassis_subsystem_mock.hpp"

using namespace aruwsrc::chassis;
using namespace aruwsrc;
using namespace testing;

#define SETUP_TEST_OBJECTS()                                         \
    Drivers drivers;                                                 \
    NiceMock<aruwsrc::mock::ChassisSubsystemMock> chassis(&drivers); \
    ChassisImuDriveCommand chassisImuDriveCommand(&drivers, &chassis);

TEST(ChassisImuDriveCommand, end__sets_des_out_0)
{
    SETUP_TEST_OBJECTS();

    EXPECT_CALL(chassis, setDesiredOutput(0, 0, 0)).Times(2);

    chassisImuDriveCommand.end(true);
    chassisImuDriveCommand.end(false);
}

TEST(ChassisImuDriveCommand, isFinished__returns_false)
{
    SETUP_TEST_OBJECTS();

    EXPECT_FALSE(chassisImuDriveCommand.isFinished());
}

static void setupUserInput(Drivers &drivers, float *userX, float *userY, float *userR)
{
    ON_CALL(drivers.controlOperatorInterface, getChassisXInput).WillByDefault(ReturnPointee(userX));
    ON_CALL(drivers.controlOperatorInterface, getChassisYInput).WillByDefault(ReturnPointee(userY));
    ON_CALL(drivers.controlOperatorInterface, getChassisRInput).WillByDefault(ReturnPointee(userR));
}

TEST(ChassisImuDriveCommand, execute__normal_rotation_translation_when_imu_not_connected)
{
    SETUP_TEST_OBJECTS();

    float userX = 0, userY = 0, userR = 0;

    ON_CALL(drivers.mpu6500, initialized).WillByDefault(Return(false));
    setupUserInput(drivers, &userX, &userY, &userR);
    ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::calculateRotationTranslationalGain(r);
    });

    std::vector<std::array<float, 3>> desiredOutputValuesToTest =
        {{0, 0, 0}, {0.5, 0.5, 0.5}, {-0.2, 0.4, 0.7}, {1, 1, 1}, {-1, -1, -1}};

    chassisImuDriveCommand.initialize();

    for (auto triplet : desiredOutputValuesToTest)
    {
        userX = triplet[0];
        userY = triplet[1];
        userR = triplet[2];
        EXPECT_CALL(
            chassis,
            setDesiredOutput(
                ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR * triplet[0],
                ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR * triplet[1],
                ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR * triplet[2]));

        chassisImuDriveCommand.execute();
    }
}

TEST(
    ChassisImuDriveCommand,
    execute__imu_setpoint_initialized_if_initialize_called_with_mpu6500_not_initialized)
{
    SETUP_TEST_OBJECTS();

    float userX = 0, userY = 0, userR = 0;
    setupUserInput(drivers, &userX, &userY, &userR);

    EXPECT_CALL(drivers.mpu6500, initialized)
        .Times(3)
        .WillOnce(Return(false))
        .WillRepeatedly(Return(true));

    EXPECT_CALL(chassis, setDesiredOutput(0, 0, 0));
    userR = 0.5;  // user R input ignored since initializing.

    chassisImuDriveCommand.initialize();  // imu not initialized
    chassisImuDriveCommand.execute();     // imu now initialized
}

TEST(
    ChassisImuDriveCommand,
    execute__imu_setpoint_used_for_rotation_compensation_if_initialize_called_with_mpu6500_initialized)
{
    SETUP_TEST_OBJECTS();

    float userX = 0, userY = 0, userR = 0;
    setupUserInput(drivers, &userX, &userY, &userR);
    ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::calculateRotationTranslationalGain(r);
    });
    ON_CALL(chassis, chassisSpeedRotationPID).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::chassisSpeedRotationPID(r);
    });
    float imuYaw = 0;
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(ReturnPointee(&imuYaw));
    ON_CALL(drivers.mpu6500, initialized).WillByDefault(Return(true));

    // imu yaw initially 0
    chassisImuDriveCommand.initialize();
    // no rotation
    EXPECT_CALL(chassis, setDesiredOutput(0, 0, 0));
    chassisImuDriveCommand.execute();
    // imu now indicates chassis is more clockwise than it should be
    imuYaw = 350;
    EXPECT_CALL(chassis, setDesiredOutput(0, 0, Ne(0)));
    chassisImuDriveCommand.execute();
    // imu now indicates chassis is more counterclockwise than it should be
    imuYaw = 10;
    EXPECT_CALL(chassis, setDesiredOutput(0, 0, Ne(0)));
    chassisImuDriveCommand.execute();
}

TEST(
    ChassisImuDriveCommand,
    execute__imu_setpoint_updates_based_on_user_input_if_initialize_called_with_mpu6500_initialized)
{
    SETUP_TEST_OBJECTS();

    float userX = 0, userY = 0, userR = 0;
    setupUserInput(drivers, &userX, &userY, &userR);
    ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::calculateRotationTranslationalGain(r);
    });
    ON_CALL(chassis, chassisSpeedRotationPID).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::chassisSpeedRotationPID(r);
    });
    float imuYaw = 0;
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(ReturnPointee(&imuYaw));
    ON_CALL(drivers.mpu6500, initialized).WillByDefault(Return(true));

    // imu yaw initially 0
    chassisImuDriveCommand.initialize();
    // no rotation
    EXPECT_CALL(chassis, setDesiredOutput(0, 0, 0));
    chassisImuDriveCommand.execute();
    // rotate chassis clockwise
    userR = 0.5;
    EXPECT_CALL(chassis, setDesiredOutput(0, 0, Ne(0)));
    chassisImuDriveCommand.execute();
    // rotate chassis counterclockwise
    userR = -1;
    EXPECT_CALL(chassis, setDesiredOutput(0, 0, Ne(0)));
    chassisImuDriveCommand.execute();
}

TEST(ChassisImuDriveCommand, execute__if_imu_err_very_large_imu_setpoint_updated)
{
    SETUP_TEST_OBJECTS();

    float userX = 0, userY = 0, userR = 0;
    setupUserInput(drivers, &userX, &userY, &userR);
    ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::calculateRotationTranslationalGain(r);
    });
    ON_CALL(chassis, chassisSpeedRotationPID).WillByDefault([&](float r) { return r; });
    float imuYaw = 0;
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(ReturnPointee(&imuYaw));
    ON_CALL(drivers.mpu6500, initialized).WillByDefault(Return(true));

    // imu yaw initially 0
    chassisImuDriveCommand.initialize();

    // very large imu yaw difference
    imuYaw = 90;
    chassisImuDriveCommand.execute();

    // setpoint is now 90 - ChassisImuDriveCommand::MAX_ROTATION_ERR
    // so if imuYaw is set to this, rotation output should be 0
    imuYaw = 90 - ChassisImuDriveCommand::MAX_ROTATION_ERR;
    EXPECT_CALL(chassis, setDesiredOutput(0, 0, 0));
    chassisImuDriveCommand.execute();
}

TEST(ChassisImuDriveCommand, execute__translational_rotation_transformed_based_on_desired_heading)
{
    SETUP_TEST_OBJECTS();

    float userX = 0, userY = 0, userR = 0;
    setupUserInput(drivers, &userX, &userY, &userR);
    ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::calculateRotationTranslationalGain(r);
    });
    ON_CALL(chassis, chassisSpeedRotationPID).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::chassisSpeedRotationPID(r);
    });
    float imuYaw = 0;
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(ReturnPointee(&imuYaw));
    ON_CALL(drivers.mpu6500, initialized).WillByDefault(Return(true));

    // imu yaw initially 0
    chassisImuDriveCommand.initialize();

    // imu off center a bit, the user rotation is put through rotation matrix s.t. forward is
    // forward relative to the desired heading
    imuYaw = 10;
    userX = 1.0f;
    userY = 1.0f;
    float xExpected = ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    float yExpected = ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    tap::algorithms::rotateVector(&xExpected, &yExpected, modm::toRadian(10));

    EXPECT_CALL(chassis, setDesiredOutput(FloatEq(xExpected), FloatEq(yExpected), _));
    chassisImuDriveCommand.execute();
}
