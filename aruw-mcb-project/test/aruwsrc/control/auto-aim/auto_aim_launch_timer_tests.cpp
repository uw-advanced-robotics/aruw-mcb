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

static constexpr uint32_t TIME_MICROS = 1'000'000;

static constexpr uint32_t DEFAULT_AGITATOR_LATENCY_MICROS = 100'000;
static constexpr uint32_t DEFAULT_FLIGHT_LATENCY_MICROS = 200'000;
static constexpr uint32_t DEFAULT_TIME_SINCE_MESSAGE_RECEIPT = 300'000;

static constexpr uint32_t SMALL_TIMING_ERROR = 10;

struct TestParams {
    uint8_t turretNumber = 0;

    uint32_t agitatorLatencyMicros = DEFAULT_AGITATOR_LATENCY_MICROS;

    uint32_t ballisticsTimeOfFlight;
    bool ballisticsSuccess = true;

    VisionCoprocessor::TurretAimData aimData;

    AutoAimLaunchTimer::LaunchInclination expectedResult;
};

class AutoAimLaunchTimerTestParameterizedFixture :public ::testing::WithParamInterface<TestParams>, public AutoAimLaunchTimerTest {
};

TEST_P(AutoAimLaunchTimerTestParameterizedFixture, getCurrentLaunchInclination_correct_result_normal_operation)
{
    auto params = GetParam();

    ClockStub clock;
    clock.time = TIME_MICROS / 1000;

    EXPECT_CALL(visionCoprocessor, getLastAimData(params.turretNumber))
        .WillOnce(ReturnPointee(&params.aimData));

    EXPECT_CALL(ballistics, computeTurretAimAngles)
        .WillOnce([&](float*, float*, float*, float *timeOfFlight) { *timeOfFlight = params.ballisticsTimeOfFlight / 1'000'000.; return params.ballisticsSuccess; } );

    AutoAimLaunchTimer timer(params.agitatorLatencyMicros, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(params.turretNumber);

    ASSERT_EQ(params.expectedResult, result);
}

static constexpr TestParams TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_ALLOWS_FIRE {
    .ballisticsTimeOfFlight = DEFAULT_FLIGHT_LATENCY_MICROS,
    .aimData {
        .hasTarget = true,
        .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
        .recommendUseTimedShots = true,
        .targetHitTimeOffset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS + DEFAULT_FLIGHT_LATENCY_MICROS,
        .targetPulseInterval = REALLY_LONG_TIME,
        .targetIntervalDuration = 2,
    },
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};

static constexpr TestParams TEST_TIMING_SHOT_IN_EARLY_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE {
    .ballisticsTimeOfFlight = DEFAULT_FLIGHT_LATENCY_MICROS - SMALL_TIMING_ERROR,
    .aimData {
        .hasTarget = true,
        .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
        .recommendUseTimedShots = true,
        .targetHitTimeOffset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS + DEFAULT_FLIGHT_LATENCY_MICROS,
        .targetPulseInterval = REALLY_LONG_TIME,
        .targetIntervalDuration = SMALL_TIMING_ERROR * 2 + 2,
    },
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};
static constexpr TestParams TEST_TIMING_SHOT_IN_LATE_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE {
    .ballisticsTimeOfFlight = DEFAULT_FLIGHT_LATENCY_MICROS + SMALL_TIMING_ERROR,
    .aimData = TEST_TIMING_SHOT_IN_EARLY_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};


static constexpr TestParams TEST_TIMING_SHOT_TOO_EARLY_IN_FIRST_WINDOW_DENIES_FIRE {
    .ballisticsTimeOfFlight = TEST_TIMING_SHOT_IN_EARLY_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE.ballisticsTimeOfFlight - 1,
    .aimData = TEST_TIMING_SHOT_IN_EARLY_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};
static constexpr TestParams TEST_TIMING_SHOT_TOO_LATE_IN_FIRST_WINDOW_DENIES_FIRE {
    .ballisticsTimeOfFlight = TEST_TIMING_SHOT_IN_LATE_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE.ballisticsTimeOfFlight + 1,
    .aimData = TEST_TIMING_SHOT_IN_LATE_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};



INSTANTIATE_TEST_CASE_P(
        AutoAimLaunchTimerTestParameterized,
        AutoAimLaunchTimerTestParameterizedFixture,
        ::testing::Values(
                TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_ALLOWS_FIRE,
                TEST_TIMING_SHOT_IN_EARLY_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE,
                TEST_TIMING_SHOT_IN_LATE_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE,
                TEST_TIMING_SHOT_TOO_EARLY_IN_FIRST_WINDOW_DENIES_FIRE,
                TEST_TIMING_SHOT_TOO_LATE_IN_FIRST_WINDOW_DENIES_FIRE
                // TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_ALLOWS_FIRE,
        ));
