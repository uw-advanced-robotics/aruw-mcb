/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/chassis/chassis_imu_drive_command.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/mock/control_operator_interface_mock.hpp"
#include "aruwsrc/mock/mecanum_chassis_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"

using namespace tap::communication::sensors::imu::mpu6500;
using namespace aruwsrc::chassis;
using namespace aruwsrc;
using namespace testing;

static constexpr float MAX_SPEED = CHASSIS_POWER_TO_MAX_SPEED_LUT[0].first;

class ChassisImuDriveCommandTest : public Test
{
protected:
    ChassisImuDriveCommandTest()
        : drivers(),
          currentSensor(
              {&drivers.analog,
               aruwsrc::chassis::CURRENT_SENSOR_PIN,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA}),
          chassis(&drivers, &currentSensor),
          controlOperatorInterface(&drivers),
          robotData{}
    {
    }

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(false));
        ON_CALL(chassis, calculateRotationTranslationalGain).WillByDefault([&](float r) {
            return chassis.HolonomicChassisSubsystem::calculateRotationTranslationalGain(r);
        });
        ON_CALL(chassis, chassisSpeedRotationPID).WillByDefault([&](float r, float d) {
            return chassis.HolonomicChassisSubsystem::chassisSpeedRotationPID(r, d);
        });

        ON_CALL(drivers.mpu6500, getYawRadians).WillByDefault(ReturnPointee(&imuYaw));
        ON_CALL(drivers.mpu6500, getImuState).WillByDefault(ReturnPointee(&imuState));
    }

    void setupUserInput(float userX, float userY, float userR)
    {
        ON_CALL(controlOperatorInterface, getChassisXInput).WillByDefault(Return(userX));
        ON_CALL(controlOperatorInterface, getChassisYInput).WillByDefault(Return(userY));
        ON_CALL(controlOperatorInterface, getChassisRInput).WillByDefault(Return(userR));
    }

    tap::Drivers drivers;
    tap::communication::sensors::current::AnalogCurrentSensor currentSensor;
    NiceMock<aruwsrc::mock::MecanumChassisSubsystemMock> chassis;
    NiceMock<aruwsrc::mock::ControlOperatorInterfaceMock> controlOperatorInterface;
    tap::communication::serial::RefSerial::Rx::RobotData robotData;
    Mpu6500::ImuState imuState = Mpu6500::ImuState::IMU_CALIBRATED;
    float imuYaw = 0;
};

using ParameterizedTuple = std::tuple<float, float, float, float>;

class ChassisImuDriveCommandNoTurretParameterizedTest
    : public ChassisImuDriveCommandTest,
      public WithParamInterface<ParameterizedTuple>
{
public:
    ChassisImuDriveCommandNoTurretParameterizedTest()
        : ChassisImuDriveCommandTest(),
          chassisImuDriveCommand(&drivers, &(controlOperatorInterface), &chassis, nullptr)
    {
    }

    void SetUp() override
    {
        ChassisImuDriveCommandTest::SetUp();
        setupUserInput(std::get<0>(GetParam()), std::get<1>(GetParam()), std::get<2>(GetParam()));
        imuYaw = std::get<3>(GetParam());
    }

    ChassisImuDriveCommand chassisImuDriveCommand;
};

TEST_P(ChassisImuDriveCommandNoTurretParameterizedTest, end__sets_des_out_0)
{
    EXPECT_CALL(chassis, setZeroRPM).Times(2);

    chassisImuDriveCommand.end(true);
    chassisImuDriveCommand.end(false);
}

TEST_P(ChassisImuDriveCommandNoTurretParameterizedTest, isFinished__returns_false)
{
    EXPECT_FALSE(chassisImuDriveCommand.isFinished());
}

TEST_P(
    ChassisImuDriveCommandNoTurretParameterizedTest,
    execute__normal_rotation_translation_when_imu_not_connected)
{
    imuState = Mpu6500::ImuState::IMU_NOT_CONNECTED;

    chassisImuDriveCommand.initialize();

    EXPECT_CALL(
        chassis,
        setDesiredOutput(
            std::get<0>(GetParam()),
            std::get<1>(GetParam()),
            std::get<2>(GetParam())));

    chassisImuDriveCommand.execute();
}

TEST_P(
    ChassisImuDriveCommandNoTurretParameterizedTest,
    execute__imu_setpoint_initialized_if_initialize_called_with_mpu6500_not_initialized)
{
    imuState = Mpu6500::ImuState::IMU_NOT_CONNECTED;

    EXPECT_CALL(chassis, setDesiredOutput(_, _, 0));

    chassisImuDriveCommand.initialize();  // imu not initialized

    imuState = Mpu6500::ImuState::IMU_CALIBRATED;

    chassisImuDriveCommand.execute();  // imu now initialized
}

TEST_P(
    ChassisImuDriveCommandNoTurretParameterizedTest,
    execute__imu_target_setpoint_difference_causes_rotation)
{
    float userR = std::get<2>(GetParam());
    if (userR < 0)
    {
        EXPECT_CALL(chassis, setDesiredOutput(_, _, Lt(0)));
    }
    else if (userR > 0)
    {
        EXPECT_CALL(chassis, setDesiredOutput(_, _, Gt(0)));
    }
    else
    {
        EXPECT_CALL(chassis, setDesiredOutput(_, _, 0));
    }

    // imuYaw same in initialize and execute, only depends on userR
    chassisImuDriveCommand.initialize();
    chassisImuDriveCommand.execute();
}

TEST_P(
    ChassisImuDriveCommandNoTurretParameterizedTest,
    execute__imu_yaw_changes_nonzero_rotation_output)
{
    // override getChassisRInput, ignore parameterized value
    ON_CALL(controlOperatorInterface, getChassisRInput).WillByDefault(Return(0));

    EXPECT_CALL(chassis, setDesiredOutput(_, _, Ne(0)));

    chassisImuDriveCommand.initialize();

    imuYaw += modm::toRadian(10);

    chassisImuDriveCommand.execute();
}

TEST_P(
    ChassisImuDriveCommandNoTurretParameterizedTest,
    execute__if_imu_err_very_large_imu_setpoint_updated)
{
    // override getChassisRInput, ignore parameterized value
    ON_CALL(controlOperatorInterface, getChassisRInput).WillByDefault(Return(0));

    imuYaw = 0;
    chassisImuDriveCommand.initialize();

    imuYaw += M_PI_2;
    chassisImuDriveCommand.execute();

    imuYaw = M_PI_2 - ChassisImuDriveCommand::MAX_ROTATION_ERR;
    float rotation = INFINITY;
    ON_CALL(chassis, setDesiredOutput).WillByDefault([&](float, float, float r) { rotation = r; });

    for (int i = 0; i < 100; i++)
    {
        chassisImuDriveCommand.execute();
    }

    // output will settle to 0
    EXPECT_NEAR(0.0f, rotation, 1E-3);
}

TEST_P(
    ChassisImuDriveCommandNoTurretParameterizedTest,
    execute__translational_rotation_transformed_based_on_desired_heading)
{
    chassisImuDriveCommand.initialize();

    imuYaw += modm::toRadian(10);

    float xExpected = std::get<0>(GetParam());
    float yExpected = std::get<1>(GetParam());
    tap::algorithms::rotateVector(&xExpected, &yExpected, -modm::toRadian(10));

    EXPECT_CALL(
        chassis,
        setDesiredOutput(FloatNear(xExpected, 1E-1), FloatNear(yExpected, 1E-1), _));

    chassisImuDriveCommand.execute();
}

INSTANTIATE_TEST_SUITE_P(
    ChassisImuDriveCommand,
    ChassisImuDriveCommandNoTurretParameterizedTest,
    Values(
        ParameterizedTuple(0, 0, 0, 0),
        ParameterizedTuple(0.5 * MAX_SPEED, 0.5 * MAX_SPEED, 0.5 * MAX_SPEED, 45),
        ParameterizedTuple(-0.2 * MAX_SPEED, 0.4 * MAX_SPEED, 0.7 * MAX_SPEED, -45),
        ParameterizedTuple(MAX_SPEED, MAX_SPEED, MAX_SPEED, 90),
        ParameterizedTuple(-MAX_SPEED, -MAX_SPEED, -MAX_SPEED, 135)));

TEST_F(ChassisImuDriveCommandTest, execute__turret_relative_when_turret_not_nullptr)
{
    NiceMock<aruwsrc::mock::TurretSubsystemMock> turret(&drivers);
    ChassisImuDriveCommand chassisImuDriveCommand(
        &drivers,
        &(controlOperatorInterface),
        &chassis,
        &turret.yawMotor);

    setupUserInput(MAX_SPEED, 0, 0);

    chassisImuDriveCommand.initialize();

    ON_CALL(turret.yawMotor, getAngleFromCenter).WillByDefault(Return(M_PI_4));
    ON_CALL(turret.yawMotor, isOnline).WillByDefault(Return(true));

    float xExpected = MAX_SPEED;
    float yExpected = 0.0f;
    tap::algorithms::rotateVector(&xExpected, &yExpected, M_PI_4);

    EXPECT_CALL(
        chassis,
        setDesiredOutput(FloatNear(xExpected, 1E-3), FloatNear(yExpected, 1E-3), _));
    chassisImuDriveCommand.execute();
}
