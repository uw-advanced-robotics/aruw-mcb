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


#include "tap/drivers.hpp"
#include "aruwsrc/control/chassis/new-chassis/wheel.hpp"
#include "aruwsrc/control/chassis/new-chassis/mecanum_wheel.hpp"
#include "tap/algorithms/cmsis_mat.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/chassis/swerve_chassis_subsystem.hpp"
#include "aruwsrc/util_macros.hpp"


using namespace aruwsrc::chassis;
using namespace testing;
using namespace tap::algorithms;

class MecanumWheelTest : public Test
{
protected:
    MecanumWheelTest()
    : mockMotor(&drivers, tap::motor::MOTOR1, CAN_BUS_MOTORS, false, "mock motor"),
    mecWheel(mockMotor, testConfig, 1)
    {
    }

    void SetUp() override {
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(testing::Return(false));
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(testing::ReturnRef(robotData));
    }

    SmoothPidConfig pidConfig = {0,
    0,
    0,
    0,
    0};

    WheelConfig testConfig = {
    .wheelPositionChassisRelativeX = 1.0f,
    .wheelPositionChassisRelativeY = 1.0f,
    .wheelOrientationChassisRelative = M_PI_2,
    .distFromCenterToWheel = sqrt(2.0f),
    .diameter = 2.0f,
    .gearRatio = 10.0f,
    .motorGearRatio = 100.0f,
    .velocityPidConfig = pidConfig,
    .isPowered = true,
    .maxWheelRPM = 1000,
    .inverted = false,
    };

    tap::Drivers drivers;
    NiceMock<tap::mock::DjiMotorMock> mockMotor;
    MecanumWheel mecWheel;

    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
};

TEST_F(MecanumWheelTest, calculate_desired_wheel_velocity_returns_desired_velocity)
    {
        modm::Pair<float, float> desiredWheelVel = mecWheel.calculateDesiredWheelVelocity(0, 0, 0);
        ASSERT_EQ(0.0f, desiredWheelVel.first);
        ASSERT_EQ(0.0f, desiredWheelVel.second);

        modm::Pair<float, float> desiredWheelVel2 = mecWheel.calculateDesiredWheelVelocity(1, 0, 0);
        ASSERT_EQ(0.0f, desiredWheelVel2.first);
        ASSERT_EQ(-1.0f, desiredWheelVel2.second);
    }

TEST_F(MecanumWheelTest, numMotorsInitializedCorrectly)
{
    EXPECT_EQ(1, mecWheel.getNumMotors());  
}

TEST_F(MecanumWheelTest, execute_wheel_velocity_calculates_correct_setpoint)
    {
        mecWheel.executeWheelVelocity(0.0f, 0.0f);
        ASSERT_EQ(0, mecWheel.getSetpoint());

        mecWheel.executeWheelVelocity(0.0f, -1.0f);
        EXPECT_EQ(-1.0f, mecWheel.getSetpoint());
    }




