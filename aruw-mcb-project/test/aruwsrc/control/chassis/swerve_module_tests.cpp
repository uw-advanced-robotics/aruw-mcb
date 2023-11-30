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

#include <gtest/gtest.h>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/dji_motor_mock.hpp"

#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "aruwsrc/util_macros.hpp"

using modm::Matrix;
using modm::Vector3f;
using tap::algorithms::getSign;
using namespace aruwsrc::chassis;
using namespace testing;

class SwerveModuleTest : public Test
{
protected:
    SwerveModuleTest()
        : driveMock(&drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS1, false, "drive mock"),
          azimuthMock(
              &drivers,
              tap::motor::MOTOR5,
              tap::can::CanBus::CAN_BUS1,
              false,
              "azimuth mock"),
          module(driveMock, azimuthMock, TEST_SWERVE_CONFIG)
    {
    }

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(testing::Return(false));
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(testing::ReturnRef(robotData));
    }

    tap::Drivers drivers;
    SwerveModuleConfig TEST_SWERVE_CONFIG;
    NiceMock<tap::mock::DjiMotorMock> driveMock;
    NiceMock<tap::mock::DjiMotorMock> azimuthMock;
    SwerveModule module;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
};

TEST_F(SwerveModuleTest, allMotorsOnline)
{
    bool driveOnline, azimuthOnline;
    ON_CALL(module.driveMotor, isMotorOnline).WillByDefault(ReturnPointee(&driveOnline));
    ON_CALL(module.azimuthMotor, isMotorOnline).WillByDefault(ReturnPointee(&azimuthOnline));

    for (int i = 0; i < 0x3; i++)
    {
        driveOnline = i & 1;
        azimuthOnline = (i >> 1) & 1;

        EXPECT_FALSE(module.allMotorsOnline());
    }

    driveOnline = true;
    azimuthOnline = true;

    EXPECT_TRUE(module.allMotorsOnline());
}

TEST_F(SwerveModuleTest, initialize)
{
    EXPECT_CALL(module.driveMotor, initialize);
    EXPECT_CALL(module.azimuthMotor, initialize);

    module.initialize();
}

TEST_F(SwerveModuleTest, getAngle)
{
    ON_CALL(module.azimuthMotor, getEncoderUnwrapped).WillByDefault(Return(0));
    EXPECT_NEAR(0, module.getAngle(), 1E-3);
}

TEST_F(SwerveModuleTest, getDriveVelocity)
{
    ON_CALL(module.driveMotor, getShaftRPM).WillByDefault(Return(0));
    EXPECT_NEAR(0, module.getDriveVelocity(), 1E-3);
}

TEST_F(SwerveModuleTest, simpleDirectionChange)
{
    EXPECT_NEAR(0, module.getRotationSetpoint(), 1E-3);
    module.calculate(1, 1, 0);
    module.scaleAndSetDesiredState(1);
    EXPECT_NEAR(M_PI_4, module.getRotationSetpoint(), 1E-3);
    EXPECT_FALSE(module.getSpeedSetpoint() < 0);

    module.calculate(0, 1, 0);
    module.scaleAndSetDesiredState(1);
    EXPECT_NEAR(M_PI_2, module.getRotationSetpoint(), 1E-3);
    EXPECT_FALSE(module.getSpeedSetpoint() < 0);
}

// @todo see todo in SwerveModule class
// TEST_F(SwerveModuleTest, reversingDirectionChange)
// {
//     module.calculate(1, 1, 0);
//     module.scaleAndSetDesiredState(1);
//     EXPECT_NEAR(M_PI_4, module.getRotationSetpoint(), 1E-3);
//     EXPECT_FALSE(module.getSpeedSetpoint() < 0);

//     module.calculate(0, -1, 0);
//     module.scaleAndSetDesiredState(1);
//     EXPECT_NEAR(M_PI_2, module.getRotationSetpoint(), 1E-3);
//     EXPECT_TRUE(module.getSpeedSetpoint() < 0);
// }

// TEST_F(SwerveModuleTest, getActualModuleVelocity)
// {
//     ON_CALL(module.azimuthMotor,
//     getEncoderUnwrapped).WillByDefault(Return(DEFAULT_SWERVE_CONFIG.azimuthMotorGearing));
//     ON_CALL(module.driveMotor, getShaftRPM).WillByDefault(Return(1));

//     modm::Matrix<float, 2, 1> vel = module.getActualModuleVelocity();
//     EXPECT_NEAR(1, vel[0][0], 1E-3);
//     EXPECT_NEAR(0, vel[1][0], 1E-3);

// }
