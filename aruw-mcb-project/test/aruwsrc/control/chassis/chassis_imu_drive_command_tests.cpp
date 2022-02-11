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
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace tap::sensors;
using namespace aruwsrc::chassis;
using namespace aruwsrc;
using namespace testing;

#define SETUP_TEST_OBJECTS_NO_TURRET()                                                  \
    Drivers drivers;                                                                    \
    tap::communication::serial::RefSerial::Rx::RobotData robotData;                     \
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));       \
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false)); \
    NiceMock<aruwsrc::mock::ChassisSubsystemMock> chassis(&drivers);                    \
    ChassisImuDriveCommand chassisImuDriveCommand(&drivers, &chassis, nullptr);

#define SETUP_TEST_OBJECTS_TURRET()                                                     \
    Drivers drivers;                                                                    \
    tap::communication::serial::RefSerial::Rx::RobotData robotData;                     \
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));       \
    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false)); \
    NiceMock<aruwsrc::mock::TurretSubsystemMock> turret(&drivers);                      \
    NiceMock<aruwsrc::mock::ChassisSubsystemMock> chassis(&drivers);                    \
    ChassisImuDriveCommand chassisImuDriveCommand(&drivers, &chassis, &turret);

TEST(ChassisImuDriveCommand, end__sets_des_out_0)
{
    SETUP_TEST_OBJECTS_NO_TURRET();

    EXPECT_CALL(chassis, setDesiredOutput(0, 0, 0)).Times(2);

    chassisImuDriveCommand.end(true);
    chassisImuDriveCommand.end(false);
}

TEST(ChassisImuDriveCommand, isFinished__returns_false)
{
    SETUP_TEST_OBJECTS_NO_TURRET();

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
    SETUP_TEST_OBJECTS_NO_TURRET();

    float userX = 0, userY = 0, userR = 0;

    ON_CALL(drivers.mpu6500, getImuState)
        .WillByDefault(Return(Mpu6500::ImuState::IMU_NOT_CONNECTED));
    setupUserInput(drivers, &userX, &userY, &userR);
    ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::calculateRotationTranslationalGain(r);
    });

    std::vector<std::array<float, 3>> desiredOutputValuesToTest =
        {{0, 0, 0}, {0.5, 0.5, 0.5}, {-0.2, 0.4, 0.7}, {1, 1, 1}, {-1, -1, -1}};

    chassisImuDriveCommand.initialize();

    for (auto triplet : desiredOutputValuesToTest)
    {
        EXPECT_CALL(
            chassis,
            setDesiredOutput(
                ChassisSubsystem::MIN_WHEEL_SPEED_SINGLE_MOTOR * triplet[0],
                ChassisSubsystem::MIN_WHEEL_SPEED_SINGLE_MOTOR * triplet[1],
                ChassisSubsystem::MIN_WHEEL_SPEED_SINGLE_MOTOR * triplet[2]));
    }

    for (auto triplet : desiredOutputValuesToTest)
    {
        userX = triplet[0];
        userY = triplet[1];
        userR = triplet[2];
        chassisImuDriveCommand.execute();
    }
}

TEST(
    ChassisImuDriveCommand,
    execute__imu_setpoint_initialized_if_initialize_called_with_mpu6500_not_initialized)
{
    SETUP_TEST_OBJECTS_NO_TURRET();

    float userX = 0, userY = 0, userR = 0;
    setupUserInput(drivers, &userX, &userY, &userR);

    Mpu6500::ImuState imuState = Mpu6500::ImuState::IMU_NOT_CONNECTED;

    ON_CALL(drivers.mpu6500, getImuState).WillByDefault([&]() { return imuState; });

    EXPECT_CALL(chassis, setDesiredOutput(0, 0, 0));

    userR = 0.5;  // user R input ignored since initializing.

    chassisImuDriveCommand.initialize();  // imu not initialized

    imuState = Mpu6500::ImuState::IMU_CALIBRATED;
    chassisImuDriveCommand.execute();  // imu now initialized
}

static void setupDefaultChassisBehavior(mock::ChassisSubsystemMock &chassis)
{
    ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::calculateRotationTranslationalGain(r);
    });
    ON_CALL(chassis, chassisSpeedRotationPID).WillByDefault([&](float r) {
        return chassis.ChassisSubsystem::chassisSpeedRotationPID(r);
    });
}

static void setupDefaultImuBehavior(aruwsrc::Drivers &drivers, float *imuYaw)
{
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(ReturnPointee(imuYaw));
    ON_CALL(drivers.mpu6500, getImuState).WillByDefault(Return(Mpu6500::ImuState::IMU_CALIBRATED));
}

TEST(ChassisImuDriveCommand, execute__imu_setpoint_target_setpoint_same_0_rotation_output)
{
    SETUP_TEST_OBJECTS_NO_TURRET();

    float userX = 0, userY = 0, userR = 0;
    float imuYaw = 0;

    setupUserInput(drivers, &userX, &userY, &userR);
    setupDefaultChassisBehavior(chassis);
    setupDefaultImuBehavior(drivers, &imuYaw);

    EXPECT_CALL(chassis, setDesiredOutput(0, 0, 0));

    // imu yaw initially 0
    chassisImuDriveCommand.initialize();

    // no rotation
    chassisImuDriveCommand.execute();
}

TEST(ChassisImuDriveCommand, execute__target_gt_actual_negative_rotation_output)
{
    SETUP_TEST_OBJECTS_NO_TURRET();

    float userX = 0, userY = 0, userR = 0;
    float imuYaw = 0;

    setupUserInput(drivers, &userX, &userY, &userR);
    setupDefaultChassisBehavior(chassis);
    setupDefaultImuBehavior(drivers, &imuYaw);

    // imu yaw initially 0
    chassisImuDriveCommand.initialize();

    userR = 1.0f;  // user rotation positive, setpoint will be gt current imu yaw.

    // since chassis rotation backward, expect desired output to be less than 0
    EXPECT_CALL(chassis, setDesiredOutput(0, 0, Lt(0)));
    chassisImuDriveCommand.execute();
}

TEST(ChassisImuDriveCommand, execute__target_lt_actual_positive_rotation_output)
{
    SETUP_TEST_OBJECTS_NO_TURRET();

    float userX = 0, userY = 0, userR = 0;
    float imuYaw = 0;

    setupUserInput(drivers, &userX, &userY, &userR);
    setupDefaultChassisBehavior(chassis);
    setupDefaultImuBehavior(drivers, &imuYaw);

    // imu yaw initially 0
    chassisImuDriveCommand.initialize();

    userR = -1.0f;  // user rotation negative, setpoint will be gt current imu yaw.

    // since chassis rotation backward, expect desired output to be less than 0
    EXPECT_CALL(chassis, setDesiredOutput(0, 0, Gt(0)));
    chassisImuDriveCommand.execute();
}

TEST(ChassisImuDriveCommand, execute__imu_yaw_changes_nonzero_rotation_output)
{
    SETUP_TEST_OBJECTS_NO_TURRET();

    float userX = 0, userY = 0, userR = 0;
    float imuYaw = 0;

    setupUserInput(drivers, &userX, &userY, &userR);
    setupDefaultChassisBehavior(chassis);
    setupDefaultImuBehavior(drivers, &imuYaw);

    // imu yaw initially 0
    chassisImuDriveCommand.initialize();

    // update imu yaw to be nonzero, now output expected to be nonzero
    imuYaw = 20;

    EXPECT_CALL(chassis, setDesiredOutput(0, 0, Ne(0)));
    chassisImuDriveCommand.execute();
}

TEST(ChassisImuDriveCommand, execute__if_imu_err_very_large_imu_setpoint_updated)
{
    SETUP_TEST_OBJECTS_NO_TURRET();

    float userX = 0, userY = 0, userR = 0;
    setupUserInput(drivers, &userX, &userY, &userR);
    setupDefaultChassisBehavior(chassis);
    float imuYaw = 0;
    setupDefaultImuBehavior(drivers, &imuYaw);

    // imu yaw initially 0
    chassisImuDriveCommand.initialize();

    // very large imu yaw difference
    imuYaw = 90;
    chassisImuDriveCommand.execute();

    // setpoint is now 90 - ChassisImuDriveCommand::MAX_ROTATION_ERR
    // so if imuYaw is set to this, rotation output should be 0
    imuYaw = 90 - ChassisImuDriveCommand::MAX_ROTATION_ERR;
    float rotation = INFINITY;
    ON_CALL(chassis, setDesiredOutput).WillByDefault([&](float, float, float r) { rotation = r; });

    for (int i = 0; i < 100; i++)
    {
        chassisImuDriveCommand.execute();
    }

    // output will settle to 0
    EXPECT_NEAR(0.0f, rotation, 1E-5);
}

TEST(ChassisImuDriveCommand, execute__translational_rotation_transformed_based_on_desired_heading)
{
    SETUP_TEST_OBJECTS_NO_TURRET();

    float userX = 0, userY = 0, userR = 0;
    setupUserInput(drivers, &userX, &userY, &userR);
    setupDefaultChassisBehavior(chassis);
    float imuYaw = 0;
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(ReturnPointee(&imuYaw));
    ON_CALL(drivers.mpu6500, getImuState).WillByDefault(Return(Mpu6500::ImuState::IMU_CALIBRATED));

    // imu yaw initially 0
    chassisImuDriveCommand.initialize();

    // imu off center a bit, the user rotation is put through rotation matrix s.t. forward is
    // forward relative to the desired heading
    imuYaw = 10;
    userX = 1.0f;
    userY = 1.0f;
    float xExpected = ChassisSubsystem::MIN_WHEEL_SPEED_SINGLE_MOTOR;
    float yExpected = ChassisSubsystem::MIN_WHEEL_SPEED_SINGLE_MOTOR;
    tap::algorithms::rotateVector(&xExpected, &yExpected, modm::toRadian(10));

    EXPECT_CALL(
        chassis,
        setDesiredOutput(FloatNear(xExpected, 1E-3), FloatNear(yExpected, 1E-3), _));
    chassisImuDriveCommand.execute();
}

TEST(ChassisImuDriveCommand, execute__turret_relative_when_turret_not_nullptr)
{
    SETUP_TEST_OBJECTS_TURRET();

    float userX = 0, userY = 0, userR = 0;
    setupUserInput(drivers, &userX, &userY, &userR);
    setupDefaultChassisBehavior(chassis);
    float imuYaw = 0;
    ON_CALL(drivers.mpu6500, getYaw).WillByDefault(ReturnPointee(&imuYaw));
    ON_CALL(drivers.mpu6500, getImuState).WillByDefault(Return(Mpu6500::ImuState::IMU_CALIBRATED));

    chassisImuDriveCommand.initialize();

    ON_CALL(turret, getYawAngleFromCenter).WillByDefault(Return(45));
    ON_CALL(turret, isOnline).WillByDefault(Return(true));

    userX = 1.0f;
    userY = 0.0f;
    float xExpected = ChassisSubsystem::MIN_WHEEL_SPEED_SINGLE_MOTOR;
    float yExpected = 0.0f;
    tap::algorithms::rotateVector(&xExpected, &yExpected, modm::toRadian(-45.0f));

    EXPECT_CALL(
        chassis,
        setDesiredOutput(FloatNear(xExpected, 1E-3), FloatNear(yExpected, 1E-3), _));
    chassisImuDriveCommand.execute();
}
