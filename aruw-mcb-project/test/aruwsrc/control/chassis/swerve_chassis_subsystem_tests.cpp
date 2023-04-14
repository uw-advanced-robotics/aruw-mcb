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

#include "aruwsrc/control/chassis/swerve_chassis_subsystem.hpp"
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

class SwerveChassisSubsystemTest : public Test
{
protected:
    SwerveChassisSubsystemTest()
        : chassis(
              &drivers,
              DEFAULT_SWERVE_CONFIG,
              DEFAULT_SWERVE_CONFIG,
              DEFAULT_SWERVE_CONFIG,
              DEFAULT_SWERVE_CONFIG)
    {
    }

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(testing::Return(false));
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(testing::ReturnRef(robotData));
    }

    tap::Drivers drivers;
    SwerveChassisSubsystem chassis;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
};

TEST_F(SwerveChassisSubsystemTest, numModulesInitializedCorrectly)
{
    EXPECT_EQ(4, chassis.NUM_MODULES);  // has to be manually changed to reflect chassis constructor
}

TEST_F(SwerveChassisSubsystemTest, getDesiredRotation_returns_desired_rotation)
{
    chassis.setDesiredOutput(0, 0, CHASSIS_VEL);
    EXPECT_NEAR(CHASSIS_VEL, chassis.getDesiredRotation(), 1E-3);
}

TEST_F(SwerveChassisSubsystemTest, setZeroRPM_doesnt_reset_desired_orientation)
{
    chassis.setDesiredOutput(0, 0, CHASSIS_VEL);
    chassis.setZeroRPM();
    EXPECT_NEAR(CHASSIS_VEL, chassis.getDesiredRotation(), 1E-3);
    modm::Matrix<float, 3, 1> desiredVelocity = chassis.getDesiredVelocityChassisRelative();
    EXPECT_NEAR(0, desiredVelocity[2][0], 1E-3);
}

TEST_F(SwerveChassisSubsystemTest, allMotorsOnline)
{
    bool online1, online2, online3, online4;
    ON_CALL(chassis.modules[0], allMotorsOnline).WillByDefault(ReturnPointee(&online1));
    ON_CALL(chassis.modules[1], allMotorsOnline).WillByDefault(ReturnPointee(&online2));
    ON_CALL(chassis.modules[2], allMotorsOnline).WillByDefault(ReturnPointee(&online3));
    ON_CALL(chassis.modules[3], allMotorsOnline).WillByDefault(ReturnPointee(&online4));

    int cases = 1;
    for (unsigned int i = 0; i < chassis.NUM_MODULES; i++)
    {
        cases *= 2;
    }
    cases -= 1;
    for (int i = 0; i < cases; i++)
    {
        online1 = i & 1;
        online2 = (i >> 1) & 1;
        online3 = (i >> 2) & 1;
        online4 = (i >> 3) & 1;

        EXPECT_FALSE(chassis.allMotorsOnline());
    }

    online1 = true;
    online2 = true;
    online3 = true;
    online4 = true;

    EXPECT_TRUE(chassis.allMotorsOnline());
}

TEST_F(SwerveChassisSubsystemTest, onHardwareTestStart_sets_desired_out_0)
{
    chassis.setDesiredOutput(1000, 1000, 1000);
    chassis.onHardwareTestStart();

    Matrix<float, 3, 1> chassiSVelocity = chassis.getDesiredVelocityChassisRelative();

    EXPECT_NEAR(0, chassiSVelocity[0][0], 1E-3);
    EXPECT_NEAR(0, chassiSVelocity[1][0], 1E-3);
    EXPECT_NEAR(0, chassiSVelocity[2][0], 1E-3);
}

TEST_F(SwerveChassisSubsystemTest, initialize)
{
    for (unsigned int i = 0; i < chassis.NUM_MODULES; i++)
    {
        EXPECT_CALL(chassis.modules[i], initialize);
    }

    chassis.initialize();
}
