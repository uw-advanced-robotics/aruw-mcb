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
#include "tap/drivers.hpp"

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

class HolonomicChassisSubsystemTest : public Test
{
protected:
    HolonomicChassisSubsystemTest() : chassis(&drivers) {}

    void SetUp() override
    {
        ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(testing::Return(false));
        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(testing::ReturnRef(robotData));
    }

    tap::Drivers drivers;
    MecanumChassisSubsystem chassis;
    tap::communication::serial::RefSerialData::Rx::RobotData robotData;
};

TEST_F(HolonomicChassisSubsystemTest, calculateRotationTranslationalGain_0_rotation)  // holonomic
{
    EXPECT_NEAR(1, chassis.calculateRotationTranslationalGain(0), 1E-3);
}

TEST_F(
    HolonomicChassisSubsystemTest,
    calculateRotationTranslationalGain_almost_0_rotation)  // holonomic
{
    EXPECT_NEAR(1, chassis.calculateRotationTranslationalGain(MIN_ROTATION_THRESHOLD), 1E-3);
    EXPECT_NEAR(1, chassis.calculateRotationTranslationalGain(-MIN_ROTATION_THRESHOLD), 1E-3);
}

TEST_F(HolonomicChassisSubsystemTest, calculateRotationTranslationalGain_max_velocity)  // holonomic
{
    EXPECT_NEAR(
        powf(MIN_ROTATION_THRESHOLD / CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second, 2.0f),
        chassis.calculateRotationTranslationalGain(CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second),
        1E-3);
}

TEST_F(
    HolonomicChassisSubsystemTest,
    calculateRotationTranslationalGain_gt_max_velocity)  // holonomic
{
    EXPECT_NEAR(
        0,
        chassis.calculateRotationTranslationalGain(
            CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second + MIN_ROTATION_THRESHOLD),
        1E-3);
}

// See the googletest cookbook for how MATCHER_P2 works
// http://google.github.io/googletest/gmock_cook_book.html
MATCHER_P2(  // holonomic
    InClosedRange,
    low,
    hi,
    std::string(negation ? "isn't" : "is") + std::string(" in range [") + PrintToString(low) +
        ", " + PrintToString(hi) + std::string("]"))
{
    return low <= arg && arg <= hi;
}

TEST_F(
    HolonomicChassisSubsystemTest,
    calculateRotationTranslationalGain_half_rotation)  // holonomic
{
    EXPECT_THAT(
        chassis.calculateRotationTranslationalGain(CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second / 2),
        InClosedRange(0.0f, 1.0f));
}

TEST_F(HolonomicChassisSubsystemTest, chassisSpeedRotationPID_basic_validation)  // holonomic
{
    EXPECT_NEAR(0, chassis.chassisSpeedRotationPID(0, 0), 1E-3);
    EXPECT_GT(chassis.chassisSpeedRotationPID(1000 * getSign(AUTOROTATION_PID_KP), 0), 0);
    EXPECT_LT(chassis.chassisSpeedRotationPID(-1000 * getSign(AUTOROTATION_PID_KP), 0), 0);
    if (AUTOROTATION_PID_MAX_D > 0)
    {
        EXPECT_GT(chassis.chassisSpeedRotationPID(0, 1000 * getSign(AUTOROTATION_PID_KD)), 0);
        EXPECT_LT(chassis.chassisSpeedRotationPID(0, -1000 * getSign(AUTOROTATION_PID_KP)), 0);
    }
}

struct VelocityGetterParam
{
    Vector3f desiredOutput;
    Vector3f expectedVelocity;
    float worldRelHeading;
};

class VelocityGetterTest : public HolonomicChassisSubsystemTest,
                           public WithParamInterface<VelocityGetterParam>
{
};

TEST_P(VelocityGetterTest, getDesiredVelocity)
{
    modm::Vector3f desiredOutput = GetParam().desiredOutput;
    modm::Vector3f expectedVelocity = GetParam().expectedVelocity;

    chassis.setDesiredOutput(desiredOutput.x, desiredOutput.y, desiredOutput.z);

    Matrix<float, 3, 1> chassisVelocity = chassis.getDesiredVelocityChassisRelative();

    EXPECT_NEAR(expectedVelocity.x, chassisVelocity[0][0], 1E-3);
    EXPECT_NEAR(expectedVelocity.y, chassisVelocity[1][0], 1E-3);
    EXPECT_NEAR(expectedVelocity.z, chassisVelocity[2][0], 1E-3);
}

TEST_P(VelocityGetterTest, getVelocityWorldRelative)
{
    modm::Matrix<float, 3, 1> expectedVelocity;
    expectedVelocity[0][0] = GetParam().expectedVelocity.x;
    expectedVelocity[1][0] = GetParam().expectedVelocity.y;
    expectedVelocity[2][0] = GetParam().expectedVelocity.z;

    modm::Matrix<float, 3, 1> worldFrameExpectedVelocity = expectedVelocity;

    float heading = GetParam().worldRelHeading;

    chassis.getVelocityWorldRelative(worldFrameExpectedVelocity, heading);

    // just a rotation matrix

    EXPECT_NEAR(
        expectedVelocity[0][0] * cosf(heading) - expectedVelocity[1][0] * sinf(heading),
        worldFrameExpectedVelocity[0][0],
        1E-3);

    EXPECT_NEAR(
        expectedVelocity[0][0] * sinf(heading) + expectedVelocity[1][0] * cosf(heading),
        worldFrameExpectedVelocity[1][0],
        1E-3);

    EXPECT_NEAR(expectedVelocity[2][0], worldFrameExpectedVelocity[2][0], 1E-3);
}

VelocityGetterParam velocityGetterValuesToTest[] = {
    {{0, 0, 0}, {0, 0, 0}, 0},
    {{0, 0, 0}, {0, 0, 0}, M_PI / 4},
    {{WHEEL_VEL, 0, 0}, {CHASSIS_VEL, 0, 0}, 0},
    {{WHEEL_VEL, 0, 0}, {CHASSIS_VEL, 0, 0}, M_PI / 3},
    {{WHEEL_VEL, 0, 0}, {CHASSIS_VEL, 0, 0}, M_PI / 2},
    {{WHEEL_VEL, 0, 0}, {CHASSIS_VEL, 0, 0}, M_PI},
    {{WHEEL_VEL, 0, 0}, {CHASSIS_VEL, 0, 0}, -M_PI / 2},
    {{0, WHEEL_VEL, 0}, {0, CHASSIS_VEL, 0}, 0},
    {{0, 0, WHEEL_VEL}, {0, 0, CHASSIS_VEL_R}, -1.5 * M_PI},
    {{WHEEL_VEL, WHEEL_VEL, 0}, {CHASSIS_VEL, CHASSIS_VEL, 0}, 1.5 * M_PI},
    {{WHEEL_VEL, WHEEL_VEL, WHEEL_VEL}, {CHASSIS_VEL, CHASSIS_VEL, CHASSIS_VEL_R}, M_PI},
    {{-WHEEL_VEL, 0, 0}, {-CHASSIS_VEL, 0, 0}, 0},
    {{0, -WHEEL_VEL, 0}, {0, -CHASSIS_VEL, 0}, 0},
    {{0, 0, -WHEEL_VEL}, {0, 0, -CHASSIS_VEL_R}, M_PI / 4},
    {{-WHEEL_VEL, -WHEEL_VEL, 0}, {-CHASSIS_VEL, -CHASSIS_VEL, 0}, M_PI / 6},
    {{WHEEL_VEL, -WHEEL_VEL, 0}, {CHASSIS_VEL, -CHASSIS_VEL, 0}, M_PI / 4},
};

INSTANTIATE_TEST_SUITE_P(
    HolonomicChassisSubsystem,
    VelocityGetterTest,
    ValuesIn(velocityGetterValuesToTest));

class MaxWheelSpeedGetterTest : public TestWithParam<std::tuple<bool, int, float>>
{
};

TEST_P(MaxWheelSpeedGetterTest, getMaxWheelSpeed)
{
    float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
        std::get<0>(GetParam()),
        std::get<1>(GetParam()));

    EXPECT_NEAR(std::get<2>(GetParam()), maxWheelSpeed, 1E-3);
}

static constexpr int POWER_TO_SPEED_LUT_ELEMENTS = MODM_ARRAY_SIZE(CHASSIS_POWER_TO_MAX_SPEED_LUT);

std::tuple<bool, int, float> maxWheelSpeedValuesToTest[] = {
    {false, 0, CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second},
    {false, CHASSIS_POWER_TO_MAX_SPEED_LUT[0].first - 1, CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second},
    {false, CHASSIS_POWER_TO_MAX_SPEED_LUT[0].first, CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second},
    {true, 0, CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second},
    {true, CHASSIS_POWER_TO_MAX_SPEED_LUT[0].first - 1, CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second},
    {true, CHASSIS_POWER_TO_MAX_SPEED_LUT[0].first, CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second},
    {true,
     CHASSIS_POWER_TO_MAX_SPEED_LUT[POWER_TO_SPEED_LUT_ELEMENTS - 1].first,
     CHASSIS_POWER_TO_MAX_SPEED_LUT[POWER_TO_SPEED_LUT_ELEMENTS - 1].second},
    {true,
     CHASSIS_POWER_TO_MAX_SPEED_LUT[POWER_TO_SPEED_LUT_ELEMENTS - 1].first + 1,
     CHASSIS_POWER_TO_MAX_SPEED_LUT[POWER_TO_SPEED_LUT_ELEMENTS - 1].second},
    {false,
     CHASSIS_POWER_TO_MAX_SPEED_LUT[POWER_TO_SPEED_LUT_ELEMENTS - 1].first,
     CHASSIS_POWER_TO_MAX_SPEED_LUT[0].second},
};

INSTANTIATE_TEST_SUITE_P(
    HolonomicChassisSubsystem,
    MaxWheelSpeedGetterTest,
    ValuesIn(maxWheelSpeedValuesToTest));

struct ActualVelocityParam
{
    int16_t lfRPM, lbRPM, rfRPM, rbRPM;
    float expectedX, expectedY, expectedR;
};

class ActualVelocityTest : public HolonomicChassisSubsystemTest,
                           public WithParamInterface<ActualVelocityParam>
{
public:
    void SetUp() override
    {
        HolonomicChassisSubsystemTest::SetUp();

        ON_CALL(chassis.leftFrontMotor, getShaftRPM).WillByDefault(Return(GetParam().lfRPM));
        ON_CALL(chassis.leftBackMotor, getShaftRPM).WillByDefault(Return(GetParam().lbRPM));
        ON_CALL(chassis.rightFrontMotor, getShaftRPM).WillByDefault(Return(GetParam().rfRPM));
        ON_CALL(chassis.rightBackMotor, getShaftRPM).WillByDefault(Return(GetParam().rbRPM));
    }
};

TEST_P(ActualVelocityTest, getActualVelocityChassisRelative)
{
    Matrix<float, 3, 1> actualVelocity = chassis.getActualVelocityChassisRelative();

    EXPECT_NEAR(GetParam().expectedX, actualVelocity[0][0], 1E-3);
    EXPECT_NEAR(GetParam().expectedY, actualVelocity[1][0], 1E-3);
    EXPECT_NEAR(GetParam().expectedR, actualVelocity[2][0], 1E-3);
}

ActualVelocityParam actualVelocityValuesToTest[] = {
    {
        .lfRPM = 0,
        .lbRPM = 0,
        .rfRPM = 0,
        .rbRPM = 0,
        .expectedX = 0,
        .expectedY = 0,
        .expectedR = 0,
    },
    {
        .lfRPM = static_cast<int16_t>(WHEEL_VEL),
        .lbRPM = static_cast<int16_t>(WHEEL_VEL),
        .rfRPM = static_cast<int16_t>(WHEEL_VEL),
        .rbRPM = static_cast<int16_t>(WHEEL_VEL),
        .expectedX = 0,
        .expectedY = 0,
        .expectedR = -CHASSIS_VEL_R,
    },
    {
        .lfRPM = static_cast<int16_t>(-WHEEL_VEL),
        .lbRPM = static_cast<int16_t>(-WHEEL_VEL),
        .rfRPM = static_cast<int16_t>(-WHEEL_VEL),
        .rbRPM = static_cast<int16_t>(-WHEEL_VEL),
        .expectedX = 0,
        .expectedY = 0,
        .expectedR = CHASSIS_VEL_R,
    },
    {
        .lfRPM = static_cast<int16_t>(WHEEL_VEL),
        .lbRPM = static_cast<int16_t>(WHEEL_VEL),
        .rfRPM = static_cast<int16_t>(-WHEEL_VEL),
        .rbRPM = static_cast<int16_t>(-WHEEL_VEL),
        .expectedX = CHASSIS_VEL,
        .expectedY = 0,
        .expectedR = 0,
    },
    {
        .lfRPM = static_cast<int16_t>(-WHEEL_VEL),
        .lbRPM = static_cast<int16_t>(-WHEEL_VEL),
        .rfRPM = static_cast<int16_t>(WHEEL_VEL),
        .rbRPM = static_cast<int16_t>(WHEEL_VEL),
        .expectedX = -CHASSIS_VEL,
        .expectedY = 0,
        .expectedR = 0,
    },
};

INSTANTIATE_TEST_SUITE_P(
    HolonomicChassisSubsystem,
    ActualVelocityTest,
    ValuesIn(actualVelocityValuesToTest));
