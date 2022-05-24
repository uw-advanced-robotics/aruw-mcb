/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/mock/hold_repeat_command_mapping_mock.hpp"
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"
#include "aruwsrc/mock/otto_ballistics_solver_mock.hpp"
#include "aruwsrc/mock/referee_feedback_friction_wheel_subsystem_mock.hpp"
#include "aruwsrc/mock/turret_subsystem_mock.hpp"
#include "tap/mock/odometry_2d_interface_mock.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/drivers.hpp"

using namespace testing;
using namespace aruwsrc::serial;
using namespace aruwsrc::control::auto_aim;
using namespace tap::arch::clock;

// 20 minutes
static constexpr uint32_t REALLY_LONG_TIME = 20 * 60 * 1'000'000;

class AutoAimLaunchTimerTest : public Test
{
protected:
    AutoAimLaunchTimerTest() : turret(&drivers), frictionWheels(&drivers), ballistics(
        drivers,
        odometry,
        turret,
        frictionWheels,
        0,
        0
    ) {};

    void SetUp() override {}

    // Contrived deps due to unfortunate mock structure
    aruwsrc::Drivers drivers;
    NiceMock<tap::mock::Odometry2DInterfaceMock> odometry;
    NiceMock<aruwsrc::mock::TurretSubsystemMock> turret;
    NiceMock<aruwsrc::mock::RefereeFeedbackFrictionWheelSubsystemMock> frictionWheels;

    NiceMock<aruwsrc::mock::VisionCoprocessorMock> visionCoprocessor;
    NiceMock<aruwsrc::mock::OttoBallisticsSolverMock> ballistics;
};

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_no_target_from_coprocessor_gives_no_target_inclination)
{
    VisionCoprocessor::TurretAimData aimData;
    aimData.hasTarget = 0;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0))
        .WillOnce(ReturnPointee(&aimData));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::NO_TARGET, result);
}

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_retrieves_data_for_specified_turret)
{
    VisionCoprocessor::TurretAimData aimData;
    aimData.hasTarget = 0;

    EXPECT_CALL(visionCoprocessor, getLastAimData(1))
        .WillOnce(ReturnPointee(&aimData));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(1);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::NO_TARGET, result);
}

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_valid_non_timed_target_returns_ungated)
{
    VisionCoprocessor::TurretAimData aimData;
    aimData.hasTarget = 1;
    aimData.recommendUseTimedShots = 0;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0))
        .WillOnce(ReturnPointee(&aimData));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::UNGATED, result);
}

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_failed_ballistics_returns_deny)
{
    VisionCoprocessor::TurretAimData aimData;
    aimData.hasTarget = 1;
    aimData.recommendUseTimedShots = 1;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0))
        .WillOnce(ReturnPointee(&aimData));

    EXPECT_CALL(ballistics, computeTurretAimAngles)
        .WillOnce([&](float*, float*, float*, float*) { return false; } );

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::GATED_DENY, result);
}


// TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_timing_exactly_on_target_upon_receipt_allows_fire)
// {
//     static constexpr uint32_t TIME_MICROS = 1'000'000;

//     static constexpr uint32_t AGITATOR_LATENCY_MICROS = 100'000;
//     static constexpr uint32_t FLIGHT_LATENCY_MICROS = 200'000;
//     static constexpr uint32_t TIME_SINCE_MESSAGE_RECEIPT = 300'000;

//     ClockStub clock;
//     clock.time = TIME_MICROS / 1000;

//     VisionCoprocessor::TurretAimData aimData;
//     aimData.hasTarget = true;
//     aimData.recommendUseTimedShots = true;
//     aimData.targetHitTimeOffset = TIME_SINCE_MESSAGE_RECEIPT + AGITATOR_LATENCY_MICROS + FLIGHT_LATENCY_MICROS;
//     aimData.targetPulseInterval = REALLY_LONG_TIME;
//     aimData.targetIntervalDuration = 2;
//     aimData.timestamp = TIME_MICROS - TIME_SINCE_MESSAGE_RECEIPT;

//     EXPECT_CALL(visionCoprocessor, getLastAimData(0))
//         .WillOnce(ReturnPointee(&aimData));

//     EXPECT_CALL(ballistics, computeTurretAimAngles)
//         .WillOnce([&](float *pitch, float *yaw, float *distance, float *timeOfFlight) { *timeOfFlight = FLIGHT_LATENCY_MICROS / 1'000'000.; return true; } );

//     AutoAimLaunchTimer timer(AGITATOR_LATENCY_MICROS, &visionCoprocessor, &ballistics);
//     auto result = timer.getCurrentLaunchInclination(0);

//     ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW, result);
// }


// TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_shot_in_early_half_of_window_allows_fire)
// {
//     static constexpr uint32_t TIME_MICROS = 1'000'000;

//     static constexpr uint32_t AGITATOR_LATENCY_MICROS = 100'000;
//     static constexpr uint32_t FLIGHT_LATENCY_MICROS = 200'000;
//     static constexpr uint32_t TIME_SINCE_MESSAGE_RECEIPT = 300'000;

//     ClockStub clock;
//     clock.time = TIME_MICROS / 1000;

//     VisionCoprocessor::TurretAimData aimData;
//     aimData.hasTarget = true;
//     aimData.recommendUseTimedShots = true;
//     aimData.targetHitTimeOffset = TIME_SINCE_MESSAGE_RECEIPT + AGITATOR_LATENCY_MICROS + FLIGHT_LATENCY_MICROS - HIT_TIME_ERROR_MICROS;
//     aimData.targetPulseInterval = REALLY_LONG_TIME;
//     aimData.targetIntervalDuration = 10;
//     aimData.timestamp = TIME_MICROS - TIME_SINCE_MESSAGE_RECEIPT;

//     EXPECT_CALL(visionCoprocessor, getLastAimData(0))
//         .WillOnce(ReturnPointee(&aimData));

//     EXPECT_CALL(ballistics, computeTurretAimAngles)
//         .WillOnce([&](float *pitch, float *yaw, float *distance, float *timeOfFlight) { *timeOfFlight = FLIGHT_LATENCY_MICROS / 1'000'000.; return true; } );

//     AutoAimLaunchTimer timer(AGITATOR_LATENCY_MICROS, &visionCoprocessor, &ballistics);
//     auto result = timer.getCurrentLaunchInclination(0);

//     ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW, result);
// }


// struct TestParameters {

// }

static constexpr uint32_t TIME_MICROS = 1'000'000;

static constexpr uint32_t AGITATOR_LATENCY_MICROS = 100'000;
static constexpr uint32_t FLIGHT_LATENCY_MICROS = 200'000;
static constexpr uint32_t TIME_SINCE_MESSAGE_RECEIPT = 300'000;

struct TestParams {
    uint8_t turretNumber = 0;

    VisionCoprocessor::TurretAimData aimData;

    uint32_t ballisticsTimeOfFlight;
    bool ballisticsSuccess = true;
};

class AutoAimLaunchTimerTestParameterizedFixture :public ::testing::TestWithParam<TestParams> {
};

TEST_P(AutoAimLaunchTimerTestParameterizedFixture, foo)
{
    auto x = GetParam();
    ASSERT_EQ(5, x.foo);
    ASSERT_EQ(x.bar, 5);
}

TestParams xxxx = { .foo = 5, .bar = xxxx.foo };
INSTANTIATE_TEST_CASE_P(
        FFFF,
        AutoAimLaunchTimerTestParameterizedFixture,
        ::testing::Values(
                xxxx
        ));
