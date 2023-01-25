/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/control/chassis/swerve_module.hpp"
#include "aruwsrc/drivers.hpp"
#include "aruwsrc/util_macros.hpp"

using modm::Matrix;
using modm::Vector3f;
using tap::algorithms::getSign;
using namespace aruwsrc::chassis;
using namespace testing;


class SwerveModuleTest : public Test
{
protected:
    SwerveModuleTest() : module(&drivers) {}

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(testing::Return(false));
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(testing::ReturnRef(robotData));
    }

    aruwsrc::Drivers drivers;
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

TEST_F(MecanumChassisSubsystemTest, getAngle)//change test value to smth more useful lol?
{
    ON_CALL(module.azimuthMotor, getEncoderUnwrapped).WillByDefault(Return(0));
    EXPECT_NEAR(0, module.getAngle(), 1E-3);
}

TEST_F(MecanumChassisSubsystemTest, getDriveVelocity)
{
    ON_CALL(module.driveMotor, getShaftRPM).WillByDefault(Return(0));
    EXPECT_NEAR(0, module.getDriveVelocity(), 1E-3);
}

