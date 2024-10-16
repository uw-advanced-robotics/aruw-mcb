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
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/chassis/mecanum_chassis_subsystem.hpp"
#include "aruwsrc/util_macros.hpp"

using modm::Matrix;
using modm::Vector3f;
using tap::algorithms::getSign;
using namespace aruwsrc::chassis;
using namespace testing;

// See this paper for equations: https://www.hindawi.com/journals/js/2015/347379/.
static constexpr float WHEEL_VEL_RPM_TO_MPS = (2.0f * M_PI * CHASSIS_GEARBOX_RATIO / 60.0f);

static constexpr float WHEEL_VEL = CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second / 3.0f;
// translational chassis velocity in m/s, if WHEEL_VEL velocity commanded in X or Y direction
static constexpr float CHASSIS_VEL = WHEEL_VEL * WHEEL_VEL_RPM_TO_MPS * WHEEL_RADIUS;
// rotational chassis velocity in rad/s, if WHEEL_VEL velocity commanded in R direction
static constexpr float A = (WIDTH_BETWEEN_WHEELS_X + WIDTH_BETWEEN_WHEELS_Y == 0)
                               ? 1
                               : 2 / (WIDTH_BETWEEN_WHEELS_X + WIDTH_BETWEEN_WHEELS_Y);
static constexpr float CHASSIS_VEL_R = WHEEL_VEL * WHEEL_VEL_RPM_TO_MPS * WHEEL_RADIUS / ::A;

class Holonomic4MotorChassisSubsystemTest : public Test
{
protected:
    Holonomic4MotorChassisSubsystemTest()
        : currentSensor(
              {&drivers.analog,
               aruwsrc::chassis::CURRENT_SENSOR_PIN,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_MV_PER_MA,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_ZERO_MA,
               aruwsrc::communication::sensors::current::ACS712_CURRENT_SENSOR_LOW_PASS_ALPHA}),
          chassis(&drivers, &currentSensor)
    {
    }

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(testing::Return(false));
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(testing::ReturnRef(robotData));
    }

    tap::Drivers drivers;
    tap::communication::sensors::current::AnalogCurrentSensor currentSensor;
    MecanumChassisSubsystem chassis;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
};

TEST_F(Holonomic4MotorChassisSubsystemTest, getDesiredRotation_returns_desired_rotation)
{
    chassis.setDesiredOutput(0, 0, CHASSIS_VEL);
    EXPECT_NEAR(CHASSIS_VEL, chassis.getDesiredRotation(), 1E-3);
}

TEST_F(Holonomic4MotorChassisSubsystemTest, setZeroRPM_doesnt_reset_desired_orientation)
{
    chassis.setDesiredOutput(0, 0, CHASSIS_VEL);
    chassis.setZeroRPM();
    EXPECT_NEAR(CHASSIS_VEL, chassis.getDesiredRotation(), 1E-3);
    modm::Matrix<float, 3, 1> desiredVelocity = chassis.getDesiredVelocityChassisRelative();
    EXPECT_NEAR(0, desiredVelocity[2][0], 1E-3);
}

TEST_F(Holonomic4MotorChassisSubsystemTest, allMotorsOnline)
{
    bool lfOnline, lbOnline, rfOnline, rbOnline;
    ON_CALL(chassis.leftFrontMotor, isMotorOnline).WillByDefault(ReturnPointee(&lfOnline));
    ON_CALL(chassis.leftBackMotor, isMotorOnline).WillByDefault(ReturnPointee(&lbOnline));
    ON_CALL(chassis.rightFrontMotor, isMotorOnline).WillByDefault(ReturnPointee(&rfOnline));
    ON_CALL(chassis.rightBackMotor, isMotorOnline).WillByDefault(ReturnPointee(&rbOnline));

    for (int i = 0; i < 0xf; i++)
    {
        lfOnline = i & 1;
        lbOnline = (i >> 1) & 1;
        rfOnline = (i >> 2) & 1;
        rbOnline = (i >> 3) & 1;

        EXPECT_FALSE(chassis.allMotorsOnline());
    }

    lfOnline = true;
    lbOnline = true;
    rfOnline = true;
    rbOnline = true;

    EXPECT_TRUE(chassis.allMotorsOnline());
}

TEST_F(Holonomic4MotorChassisSubsystemTest, getLeftFrontRpmActual)
{
    ON_CALL(chassis.leftFrontMotor, getShaftRPM).WillByDefault(Return(1000));
    EXPECT_NEAR(1000, chassis.getLeftFrontRpmActual(), 1E-3);
}

TEST_F(Holonomic4MotorChassisSubsystemTest, getLeftBackRpmActual)
{
    ON_CALL(chassis.leftBackMotor, getShaftRPM).WillByDefault(Return(1000));
    EXPECT_NEAR(1000, chassis.getLeftBackRpmActual(), 1E-3);
}

TEST_F(Holonomic4MotorChassisSubsystemTest, getRightFrontRpmActual)
{
    ON_CALL(chassis.rightFrontMotor, getShaftRPM).WillByDefault(Return(1000));
    EXPECT_NEAR(1000, chassis.getRightFrontRpmActual(), 1E-3);
}

TEST_F(Holonomic4MotorChassisSubsystemTest, getRightBackRpmActual)
{
    ON_CALL(chassis.rightBackMotor, getShaftRPM).WillByDefault(Return(1000));
    EXPECT_NEAR(1000, chassis.getRightBackRpmActual(), 1E-3);
}

TEST_F(Holonomic4MotorChassisSubsystemTest, initialize)
{
    EXPECT_CALL(chassis.leftFrontMotor, initialize);
    EXPECT_CALL(chassis.leftBackMotor, initialize);
    EXPECT_CALL(chassis.rightFrontMotor, initialize);
    EXPECT_CALL(chassis.rightBackMotor, initialize);

    chassis.initialize();
}
