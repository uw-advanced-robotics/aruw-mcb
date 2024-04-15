/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/hold_repeat_command_mapping_mock.hpp"
#include "tap/mock/odometry_2d_interface_mock.hpp"

#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/mock/otto_ballistics_solver_mock.hpp"
#include "aruwsrc/mock/referee_feedback_friction_wheel_subsystem_mock.hpp"
#include "aruwsrc/mock/robot_turret_subsystem_mock.hpp"
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"

using namespace testing;
using namespace aruwsrc::serial;
using namespace aruwsrc::control::auto_aim;
using namespace aruwsrc::algorithms;
using namespace tap::arch::clock;

// 20 minutes
static constexpr uint32_t REALLY_LONG_TIME = 20 * 60 * 1'000'000;

class AutoAimLaunchTimerTest : public Test
{
protected:
    AutoAimLaunchTimerTest()
        : frictionWheels(&drivers),
          visionCoprocessor(&drivers),
          turretSubsystem(&drivers),
          ballistics(visionCoprocessor, odometry, turretSubsystem, frictionWheels, 0, 0){};

    void SetUp() override {}

    // Contrived deps due to unfortunate mock structure
    tap::Drivers drivers;
    NiceMock<tap::mock::Odometry2DInterfaceMock> odometry;
    NiceMock<aruwsrc::mock::RefereeFeedbackFrictionWheelSubsystemMock> frictionWheels;
    NiceMock<aruwsrc::mock::VisionCoprocessorMock> visionCoprocessor;
    NiceMock<aruwsrc::mock::RobotTurretSubsystemMock> turretSubsystem;
    NiceMock<aruwsrc::mock::OttoBallisticsSolverMock> ballistics;
};

TEST_F(
    AutoAimLaunchTimerTest,
    getCurrentLaunchInclination_no_target_from_coprocessor_gives_no_target_inclination)
{
    VisionCoprocessor::TurretAimData aimData;
    aimData.pva.updated = 0;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::NO_TARGET, result);
}

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_retrieves_data_for_specified_turret)
{
    VisionCoprocessor::TurretAimData aimData;
    aimData.pva.updated = 0;

    EXPECT_CALL(visionCoprocessor, getLastAimData(1)).WillOnce(ReturnPointee(&aimData));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(1);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::NO_TARGET, result);
}

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_valid_non_timed_target_returns_ungated)
{
    VisionCoprocessor::TurretAimData aimData;
    aimData.pva.updated = 1;
    aimData.timing.updated = 0;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::UNGATED, result);
}

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_zero_interval_returns_deny)
{
    VisionCoprocessor::TurretAimData aimData;
    aimData.pva.updated = 1;
    aimData.timing.updated = 1;
    aimData.timing.offset = 100;
    aimData.timing.duration = 100;
    aimData.timing.pulseInterval = 0;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    EXPECT_CALL(ballistics, computeTurretAimAngles).Times(0);

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::GATED_DENY, result);
}

static constexpr uint32_t TIME_MICROS = 1'000'000;

static constexpr uint32_t DEFAULT_AGITATOR_LATENCY_MICROS = 100'000;
static constexpr uint32_t DEFAULT_FLIGHT_LATENCY_MICROS = 200'000;
static constexpr uint32_t DEFAULT_TIME_SINCE_MESSAGE_RECEIPT = 300'000;

static constexpr uint32_t SMALL_TIMING_ERROR = 10;

// Ballistics output in floating-point loses a small amount of precision. If floating-point error
// were removed, this factor could be zero.
static constexpr uint32_t FLOATING_POINT_FUDGE_MICROS = 1;

namespace auto_aim  // Must be in a namespace so the operator<< can be discovered by googletest
{
struct TestParamsPositionData
{
    bool updated;
};
struct TestParamsTimingData
{
    uint32_t duration;
    uint32_t pulseInterval;
    uint32_t offset;
    bool updated;
};

struct TestParamsAimData
{
    TestParamsPositionData pva;
    uint32_t timestamp;
    TestParamsTimingData timing;

    friend std::ostream& operator<<(std::ostream& os, const TestParamsAimData& p)
    {
        return os << "{" << p.pva.updated << ", " << p.timestamp << ", " << p.timing.updated << ", "
                  << p.timing.offset << ", " << p.timing.pulseInterval << ", "
                  << p.timing.pulseInterval << "}";
    }
};

struct TestParams
{
    uint8_t turretNumber = 0;

    uint32_t agitatorLatencyMicros = DEFAULT_AGITATOR_LATENCY_MICROS;

    bool ballisticsSuccess = true;
    uint32_t ballisticsTimeOfFlight;

    TestParamsAimData aimData;

    AutoAimLaunchTimer::LaunchInclination expectedResult;

    friend std::ostream& operator<<(std::ostream& os, const TestParams& p)
    {
        return os << "{" << int(p.turretNumber) << ", " << p.agitatorLatencyMicros << ", "
                  << p.ballisticsSuccess << ", " << p.ballisticsTimeOfFlight << ", " << p.aimData
                  << uint8_t(p.expectedResult) << "}";
    }
};
}  // namespace auto_aim

using namespace auto_aim;

class AutoAimLaunchTimerTestParameterizedFixture : public ::testing::WithParamInterface<TestParams>,
                                                   public AutoAimLaunchTimerTest
{
};

TEST_P(
    AutoAimLaunchTimerTestParameterizedFixture,
    getCurrentLaunchInclination_correct_result_normal_operation)
{
    auto params = GetParam();

    ClockStub clock;
    clock.time = TIME_MICROS / 1000;

    VisionCoprocessor::TurretAimData aimData = {
        .pva =
            {
                .firerate{VisionCoprocessor::FireRate::ZERO},

                .xPos{0},
                .yPos{0},
                .zPos{0},

                .xVel{0},
                .yVel{0},
                .zVel{0},

                .xAcc{0},
                .yAcc{0},
                .zAcc{0},

                .theta{0},
                .omega{0},

                .rad0{0},
                .rad1{0},
                .plateHeights{0, 0, 0, 0},

                .updated{params.aimData.pva.updated},
            },
        .timestamp{params.aimData.timestamp},

        .timing =
            {
                .offset{params.aimData.timing.offset},
                .pulseInterval{params.aimData.timing.pulseInterval},
                .duration{params.aimData.timing.duration},
                .updated{params.aimData.timing.updated},
            },
    };
    EXPECT_CALL(visionCoprocessor, getLastAimData(params.turretNumber))
        .WillOnce(ReturnPointee(&aimData));

    std::optional<OttoBallisticsSolver::BallisticsSolution> ballisticsResult;
    if (params.ballisticsSuccess)
    {
        ballisticsResult = {
            .pitchAngle{0},
            .yawAngle{0},
            .distance{0},
            .timeOfFlight = params.ballisticsTimeOfFlight / 1'000'000.f,
        };
    }
    else
    {
        ballisticsResult = std::nullopt;
    }

    EXPECT_CALL(ballistics, computeTurretAimAngles).WillOnce(Return(ballisticsResult));

    AutoAimLaunchTimer timer(params.agitatorLatencyMicros, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(params.turretNumber);

    ASSERT_EQ(params.expectedResult, result);
}

static constexpr TestParams TEST_FAILED_BALISTICS_DENIES_FIRE{
    .ballisticsSuccess = false,
    .ballisticsTimeOfFlight = DEFAULT_FLIGHT_LATENCY_MICROS,
    .aimData{
        .pva{
            .updated = true,
        },
        .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
        .timing{
            .duration = 2,
            .pulseInterval = REALLY_LONG_TIME,
            .offset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS +
                      DEFAULT_FLIGHT_LATENCY_MICROS,
            .updated = true,
        }},
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};

// First window: exactly on target
static constexpr TestParams TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_NARROW_ALLOWS_FIRE{
    .ballisticsTimeOfFlight = DEFAULT_FLIGHT_LATENCY_MICROS,
    .aimData{
        .pva{.updated = true},
        .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
        .timing{
            .duration = 1,
            .pulseInterval = REALLY_LONG_TIME,
            .offset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS +
                      DEFAULT_FLIGHT_LATENCY_MICROS,
            .updated = true}},
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};
static constexpr TestParams TEST_TIMING_ONE_MICROSECOND_EARLY_IN_FIRST_WINDOW_NARROW_DENIES_FIRE{
    .ballisticsTimeOfFlight =
        TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_NARROW_ALLOWS_FIRE.ballisticsTimeOfFlight - 1,
    .aimData = TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_NARROW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};
static constexpr TestParams TEST_TIMING_ONE_MICROSECOND_LATE_IN_FIRST_WINDOW_NARROW_DENIES_FIRE{
    .ballisticsTimeOfFlight =
        TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_NARROW_ALLOWS_FIRE.ballisticsTimeOfFlight + 1,
    .aimData = TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_NARROW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};
static constexpr TestParams TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_WIDE_ALLOWS_FIRE{
    .ballisticsTimeOfFlight = DEFAULT_FLIGHT_LATENCY_MICROS,
    .aimData{
        .pva{
            .updated = true,
        },
        .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
        .timing{
            .duration = 600'000,
            .pulseInterval = REALLY_LONG_TIME,
            .offset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS +
                      DEFAULT_FLIGHT_LATENCY_MICROS,
            .updated = true}},
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};

static constexpr TestParams TEST_TIMING_WITHIN_WINDOW_LARGER_THAN_INTERVAL_ALLOWS_FIRE{
    .ballisticsTimeOfFlight = DEFAULT_FLIGHT_LATENCY_MICROS,
    .aimData{
        .pva{
            .updated = true,
        },
        .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
        .timing{
            .duration = 600'000,
            .pulseInterval = 100'000,
            .offset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS +
                      DEFAULT_FLIGHT_LATENCY_MICROS,
            .updated = true,
        }},
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};

// First window: edge cases
static constexpr TestParams TEST_TIMING_SHOT_IN_EARLY_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE{
    .ballisticsTimeOfFlight = DEFAULT_FLIGHT_LATENCY_MICROS - SMALL_TIMING_ERROR,
    .aimData{
        .pva{
            .updated = true,
        },
        .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
        .timing{
            .duration = SMALL_TIMING_ERROR * 2 + 1,
            .pulseInterval = REALLY_LONG_TIME,
            .offset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS +
                      DEFAULT_FLIGHT_LATENCY_MICROS,
            .updated = true,
        }},
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};
static constexpr TestParams TEST_TIMING_SHOT_IN_LATE_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE{
    .ballisticsTimeOfFlight = DEFAULT_FLIGHT_LATENCY_MICROS + SMALL_TIMING_ERROR,
    .aimData = TEST_TIMING_SHOT_IN_EARLY_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};

static constexpr TestParams TEST_TIMING_SHOT_TOO_EARLY_IN_FIRST_WINDOW_DENIES_FIRE{
    .ballisticsTimeOfFlight =
        TEST_TIMING_SHOT_IN_EARLY_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE.ballisticsTimeOfFlight - 1,
    .aimData = TEST_TIMING_SHOT_IN_EARLY_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};
static constexpr TestParams TEST_TIMING_SHOT_TOO_LATE_IN_FIRST_WINDOW_DENIES_FIRE{
    .ballisticsTimeOfFlight =
        TEST_TIMING_SHOT_IN_LATE_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE.ballisticsTimeOfFlight + 1,
    .aimData = TEST_TIMING_SHOT_IN_LATE_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};

// Second window: exactly on target
static constexpr uint32_t LARGE_PULSE_INTERVAL_MICROS = 600'000;
static constexpr TestParams TEST_TIMING_EXACTLY_ON_TARGET_IN_SECOND_WINDOW_NARROW_ALLOWS_FIRE{
    .ballisticsTimeOfFlight = LARGE_PULSE_INTERVAL_MICROS + DEFAULT_FLIGHT_LATENCY_MICROS,
    .aimData{
        .pva{
            .updated = true,
        },
        .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
        .timing{
            .duration = 1,
            .pulseInterval = LARGE_PULSE_INTERVAL_MICROS,
            .offset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS +
                      DEFAULT_FLIGHT_LATENCY_MICROS,
            .updated = true,
        }},
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};
static constexpr TestParams TEST_TIMING_ONE_MICROSECOND_EARLY_IN_SECOND_WINDOW_NARROW_DENIES_FIRE{
    .ballisticsTimeOfFlight =
        TEST_TIMING_EXACTLY_ON_TARGET_IN_SECOND_WINDOW_NARROW_ALLOWS_FIRE.ballisticsTimeOfFlight -
        1,
    .aimData = TEST_TIMING_EXACTLY_ON_TARGET_IN_SECOND_WINDOW_NARROW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};
static constexpr TestParams TEST_TIMING_ONE_MICROSECOND_LATE_IN_SECOND_WINDOW_NARROW_DENIES_FIRE{
    .ballisticsTimeOfFlight =
        TEST_TIMING_EXACTLY_ON_TARGET_IN_SECOND_WINDOW_NARROW_ALLOWS_FIRE.ballisticsTimeOfFlight +
        1,
    .aimData = TEST_TIMING_EXACTLY_ON_TARGET_IN_SECOND_WINDOW_NARROW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};

// Second window: edge cases
static constexpr TestParams TEST_TIMING_SHOT_IN_EARLY_HALF_OF_SECOND_WINDOW_ALLOWS_FIRE{
    .ballisticsTimeOfFlight = LARGE_PULSE_INTERVAL_MICROS + DEFAULT_FLIGHT_LATENCY_MICROS -
                              SMALL_TIMING_ERROR + FLOATING_POINT_FUDGE_MICROS,
    .aimData{
        .pva{
            .updated = true,
        },
        .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
        .timing{
            .duration = SMALL_TIMING_ERROR * 2 + 1,
            .pulseInterval = LARGE_PULSE_INTERVAL_MICROS,
            .offset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS +
                      DEFAULT_FLIGHT_LATENCY_MICROS,
            .updated = true,
        }},
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};
static constexpr TestParams TEST_TIMING_SHOT_IN_LATE_HALF_OF_SECOND_WINDOW_ALLOWS_FIRE{
    .ballisticsTimeOfFlight = LARGE_PULSE_INTERVAL_MICROS + DEFAULT_FLIGHT_LATENCY_MICROS +
                              SMALL_TIMING_ERROR + FLOATING_POINT_FUDGE_MICROS,
    .aimData = TEST_TIMING_SHOT_IN_EARLY_HALF_OF_SECOND_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};

static constexpr TestParams TEST_TIMING_SHOT_TOO_EARLY_IN_SECOND_WINDOW_DENIES_FIRE{
    .ballisticsTimeOfFlight =
        TEST_TIMING_SHOT_IN_EARLY_HALF_OF_SECOND_WINDOW_ALLOWS_FIRE.ballisticsTimeOfFlight - 1,
    .aimData = TEST_TIMING_SHOT_IN_EARLY_HALF_OF_SECOND_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};
static constexpr TestParams TEST_TIMING_SHOT_TOO_LATE_IN_SECOND_WINDOW_DENIES_FIRE{
    .ballisticsTimeOfFlight =
        TEST_TIMING_SHOT_IN_LATE_HALF_OF_SECOND_WINDOW_ALLOWS_FIRE.ballisticsTimeOfFlight + 1,
    .aimData = TEST_TIMING_SHOT_IN_LATE_HALF_OF_SECOND_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};

// Third window: edge cases
static constexpr TestParams TEST_TIMING_SHOT_IN_EARLY_HALF_OF_THIRD_WINDOW_ALLOWS_FIRE{
    .ballisticsTimeOfFlight = LARGE_PULSE_INTERVAL_MICROS * 2 + DEFAULT_FLIGHT_LATENCY_MICROS -
                              SMALL_TIMING_ERROR + FLOATING_POINT_FUDGE_MICROS,
    .aimData{
        .pva{
            .updated = true,
        },
        .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
        .timing{
            .duration = SMALL_TIMING_ERROR * 2 + 1,
            .pulseInterval = LARGE_PULSE_INTERVAL_MICROS,
            .offset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS +
                      DEFAULT_FLIGHT_LATENCY_MICROS,
            .updated = true,
        }},
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};
static constexpr TestParams TEST_TIMING_SHOT_IN_LATE_HALF_OF_THIRD_WINDOW_ALLOWS_FIRE{
    .ballisticsTimeOfFlight = LARGE_PULSE_INTERVAL_MICROS * 2 + DEFAULT_FLIGHT_LATENCY_MICROS +
                              SMALL_TIMING_ERROR + FLOATING_POINT_FUDGE_MICROS,
    .aimData = TEST_TIMING_SHOT_IN_EARLY_HALF_OF_THIRD_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
};

static constexpr TestParams TEST_TIMING_SHOT_TOO_EARLY_IN_THIRD_WINDOW_DENIES_FIRE{
    .ballisticsTimeOfFlight =
        TEST_TIMING_SHOT_IN_EARLY_HALF_OF_THIRD_WINDOW_ALLOWS_FIRE.ballisticsTimeOfFlight - 1,
    .aimData = TEST_TIMING_SHOT_IN_EARLY_HALF_OF_THIRD_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};
static constexpr TestParams TEST_TIMING_SHOT_TOO_LATE_IN_THIRD_WINDOW_DENIES_FIRE{
    .ballisticsTimeOfFlight =
        TEST_TIMING_SHOT_IN_LATE_HALF_OF_THIRD_WINDOW_ALLOWS_FIRE.ballisticsTimeOfFlight + 1,
    .aimData = TEST_TIMING_SHOT_IN_LATE_HALF_OF_THIRD_WINDOW_ALLOWS_FIRE.aimData,
    .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
};

// Second window: small pulse interval less than latencies
static constexpr uint32_t SMALL_PULSE_INTERVAL_MICROS = 100'000;
static constexpr TestParams
    TEST_TIMING_SHOT_IN_EARLY_HALF_OF_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_ALLOWS_FIRE{
        .ballisticsTimeOfFlight = SMALL_PULSE_INTERVAL_MICROS + DEFAULT_FLIGHT_LATENCY_MICROS -
                                  SMALL_TIMING_ERROR + FLOATING_POINT_FUDGE_MICROS,
        .aimData{
            .pva{
                .updated = true,
            },
            .timestamp = TIME_MICROS - DEFAULT_TIME_SINCE_MESSAGE_RECEIPT,
            .timing{
                .duration = SMALL_TIMING_ERROR * 2 + 1,
                .pulseInterval = SMALL_PULSE_INTERVAL_MICROS,
                .offset = DEFAULT_TIME_SINCE_MESSAGE_RECEIPT + DEFAULT_AGITATOR_LATENCY_MICROS +
                          DEFAULT_FLIGHT_LATENCY_MICROS,
                .updated = true,
            }},
        .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
    };
static constexpr TestParams
    TEST_TIMING_SHOT_IN_LATE_HALF_OF_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_ALLOWS_FIRE{
        .ballisticsTimeOfFlight =
            SMALL_PULSE_INTERVAL_MICROS + DEFAULT_FLIGHT_LATENCY_MICROS + SMALL_TIMING_ERROR,
        .aimData =
            TEST_TIMING_SHOT_IN_EARLY_HALF_OF_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_ALLOWS_FIRE
                .aimData,
        .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW,
    };

static constexpr TestParams
    TEST_TIMING_SHOT_TOO_EARLY_IN_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_DENIES_FIRE{
        .ballisticsTimeOfFlight =
            TEST_TIMING_SHOT_IN_EARLY_HALF_OF_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_ALLOWS_FIRE
                .ballisticsTimeOfFlight -
            2,
        .aimData =
            TEST_TIMING_SHOT_IN_EARLY_HALF_OF_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_ALLOWS_FIRE
                .aimData,
        .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
    };
static constexpr TestParams
    TEST_TIMING_SHOT_TOO_LATE_IN_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_DENIES_FIRE{
        .ballisticsTimeOfFlight =
            TEST_TIMING_SHOT_IN_LATE_HALF_OF_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_ALLOWS_FIRE
                .ballisticsTimeOfFlight +
            2,
        .aimData =
            TEST_TIMING_SHOT_IN_LATE_HALF_OF_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_ALLOWS_FIRE
                .aimData,
        .expectedResult = AutoAimLaunchTimer::LaunchInclination::GATED_DENY,
    };

INSTANTIATE_TEST_CASE_P(
    AutoAimLaunchTimerTestParameterized,
    AutoAimLaunchTimerTestParameterizedFixture,
    ::testing::Values(
        TEST_FAILED_BALISTICS_DENIES_FIRE,

        TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_NARROW_ALLOWS_FIRE,
        TEST_TIMING_ONE_MICROSECOND_EARLY_IN_FIRST_WINDOW_NARROW_DENIES_FIRE,
        TEST_TIMING_ONE_MICROSECOND_LATE_IN_FIRST_WINDOW_NARROW_DENIES_FIRE,
        TEST_TIMING_EXACTLY_ON_TARGET_IN_FIRST_WINDOW_WIDE_ALLOWS_FIRE,
        TEST_TIMING_WITHIN_WINDOW_LARGER_THAN_INTERVAL_ALLOWS_FIRE,

        TEST_TIMING_SHOT_IN_EARLY_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE,
        TEST_TIMING_SHOT_IN_LATE_HALF_OF_FIRST_WINDOW_ALLOWS_FIRE,
        TEST_TIMING_SHOT_TOO_EARLY_IN_FIRST_WINDOW_DENIES_FIRE,
        TEST_TIMING_SHOT_TOO_LATE_IN_FIRST_WINDOW_DENIES_FIRE,

        TEST_TIMING_EXACTLY_ON_TARGET_IN_SECOND_WINDOW_NARROW_ALLOWS_FIRE,
        TEST_TIMING_ONE_MICROSECOND_EARLY_IN_SECOND_WINDOW_NARROW_DENIES_FIRE,
        TEST_TIMING_ONE_MICROSECOND_LATE_IN_SECOND_WINDOW_NARROW_DENIES_FIRE,

        TEST_TIMING_SHOT_IN_EARLY_HALF_OF_SECOND_WINDOW_ALLOWS_FIRE,
        TEST_TIMING_SHOT_IN_LATE_HALF_OF_SECOND_WINDOW_ALLOWS_FIRE,
        TEST_TIMING_SHOT_TOO_EARLY_IN_SECOND_WINDOW_DENIES_FIRE,
        TEST_TIMING_SHOT_TOO_LATE_IN_SECOND_WINDOW_DENIES_FIRE,

        TEST_TIMING_SHOT_IN_EARLY_HALF_OF_THIRD_WINDOW_ALLOWS_FIRE,
        TEST_TIMING_SHOT_IN_LATE_HALF_OF_THIRD_WINDOW_ALLOWS_FIRE,
        TEST_TIMING_SHOT_TOO_EARLY_IN_THIRD_WINDOW_DENIES_FIRE,
        TEST_TIMING_SHOT_TOO_LATE_IN_THIRD_WINDOW_DENIES_FIRE,

        TEST_TIMING_SHOT_IN_EARLY_HALF_OF_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_ALLOWS_FIRE,
        TEST_TIMING_SHOT_IN_LATE_HALF_OF_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_ALLOWS_FIRE,
        TEST_TIMING_SHOT_TOO_EARLY_IN_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_DENIES_FIRE,
        TEST_TIMING_SHOT_TOO_LATE_IN_SECOND_WINDOW_WITH_HIGH_FREQUENCY_PULSE_DENIES_FIRE));
