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

#include "aruwsrc/algorithms/ballistics/cv_ballistics_solver.hpp"
#include "aruwsrc/control/auto-aim/auto_aim_launch_timer.hpp"
#include "aruwsrc/mock/cv_ballistics_solver_mock.hpp"
#include "aruwsrc/mock/referee_feedback_friction_wheel_subsystem_mock.hpp"
#include "aruwsrc/mock/transformer_interface_mock.hpp"
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"

using namespace testing;
using namespace aruwsrc::communication::serial;
using namespace aruwsrc::control::auto_aim;
using namespace aruwsrc::algorithms::ballistics;
using namespace tap::arch::clock;

// 20 minutes
static constexpr uint32_t REALLY_LONG_TIME = 20 * 60 * 1'000'000;

class AutoAimLaunchTimerTest : public Test
{
protected:
    AutoAimLaunchTimerTest()
        : leftFlywheel(
              &drivers,
              tap::motor::MOTOR1,
              tap::can::CanBus::CAN_BUS1,
              true,
              "Left flywheel",
              false),
          rightFlywheel(
              &drivers,
              tap::motor::MOTOR2,
              tap::can::CanBus::CAN_BUS1,
              false,
              "Right flywheel",
              false),
          worldToTurretYaw(0, 0, 0, 0, 0, 0),
          frictionWheels(
              &drivers,
              std::array<tap::motor::MotorInterface*, 2>{{&leftFlywheel, &rightFlywheel}}),
          visionCoprocessor(&drivers),
          ballistics(
              // hack to set up default return transformer return value before ballistics
              // constructor uses it
              [this]() -> auto& {
                  ON_CALL(transformer, getWorldToTurretYaw)
                      .WillByDefault(testing::ReturnRef(worldToTurretYaw));
                  return visionCoprocessor;
              }(),
              transformer,
              frictionWheels){};

    void SetUp() override {}

    // Contrived deps due to unfortunate mock structure
    tap::Drivers drivers;
    NiceMock<tap::mock::DjiMotorMock> leftFlywheel;
    NiceMock<tap::mock::DjiMotorMock> rightFlywheel;
    tap::algorithms::transforms::Transform worldToTurretYaw;
    NiceMock<aruwsrc::mock::TransformerInterfaceMock> transformer;
    NiceMock<aruwsrc::mock::RefereeFeedbackFrictionWheelSubsystemMock> frictionWheels;
    NiceMock<aruwsrc::mock::VisionCoprocessorMock> visionCoprocessor;
    NiceMock<aruwsrc::mock::CvBallisticsSolverMock> ballistics;
};

TEST_F(
    AutoAimLaunchTimerTest,
    getCurrentLaunchInclination_no_target_from_coprocessor_gives_no_target_inclination)
{
    VisionCoprocessor::TurretAimData aimData = {};
    aimData.targetState.updated = 0;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::NO_TARGET, result);
}

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_retrieves_data_for_specified_turret)
{
    VisionCoprocessor::TurretAimData aimData = {};
    aimData.targetState.updated = 0;

    EXPECT_CALL(visionCoprocessor, getLastAimData(1)).WillOnce(ReturnPointee(&aimData));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(1);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::NO_TARGET, result);
}

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_valid_non_timed_target_returns_ungated)
{
    VisionCoprocessor::TurretAimData aimData = {};
    aimData.targetState.updated = 1;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::NO_TARGET, result);
}

TEST_F(AutoAimLaunchTimerTest, getCurrentLaunchInclination_zero_interval_returns_deny)
{
    VisionCoprocessor::TurretAimData aimData = {};
    aimData.targetState.updated = 1;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    std::optional<CvBallisticsSolver::BallisticsSolution> solution({
        .pitchAngle = 0,
        .yawAngle = 0,
        .yawVel = 0,
        .yawAcc = 0,
        .distance = 0,
        .timeOfFlight = 0,
        .activePlateIndex = 0,
        .shotWindowValid = true,
        .shotWindowCenter = 0,
        .shotWindowHalfWidth = 0,
    });

    EXPECT_CALL(ballistics, computeTurretAimAngles).Times(1).WillOnce(ReturnPointee(&solution));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::GATED_DENY, result);
}

TEST_F(AutoAimLaunchTimerTest, pulse_estimation_jitter_aim_returns_ungated)
{
    VisionCoprocessor::TurretAimData aimData = {};
    aimData.targetState.updated = 1;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    CvBallisticsSolver::BallisticsSolution solution{
        .pitchAngle = 0,
        .yawAngle = 0,
        .yawVel = 0,
        .yawAcc = 0,
        .distance = 5.0f,
        .timeOfFlight = 0.2f,
        .activePlateIndex = 0,
        .shotWindowValid = false,  // Jitter aim mode
        .shotWindowCenter = 0,
        .shotWindowHalfWidth = 0,
    };
    EXPECT_CALL(ballistics, computeTurretAimAngles).WillOnce(Return(solution));

    AutoAimLaunchTimer timer(100, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::UNGATED, result);
}

TEST_F(AutoAimLaunchTimerTest, pulse_estimation_within_window_allows_fire)
{
    ClockStub clock;
    clock.time = 500;  // 500ms

    VisionCoprocessor::TurretAimData aimData = {};
    aimData.targetState.updated = 1;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    CvBallisticsSolver::BallisticsSolution solution{
        .pitchAngle = 0,
        .yawAngle = 0,
        .yawVel = 0,
        .yawAcc = 0,
        .distance = 5.0f,
        .timeOfFlight = 0.2f,
        .activePlateIndex = 1,
        .shotWindowValid = true,
        .shotWindowCenter = clock.time * 1000,
        .shotWindowHalfWidth = 100'000,  // 100ms
    };
    EXPECT_CALL(ballistics, computeTurretAimAngles).WillOnce(Return(solution));

    AutoAimLaunchTimer timer(0, &visionCoprocessor, &ballistics);  // No agitator delay
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW, result);
}

TEST_F(AutoAimLaunchTimerTest, pulse_estimation_before_window_denies_fire)
{
    ClockStub clock;
    clock.time = 300;  // 300ms

    VisionCoprocessor::TurretAimData aimData = {};
    aimData.targetState.updated = 1;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    CvBallisticsSolver::BallisticsSolution solution{
        .pitchAngle = 0,
        .yawAngle = 0,
        .yawVel = 0,
        .yawAcc = 0,
        .distance = 5.0f,
        .timeOfFlight = 0.2f,
        .activePlateIndex = 1,
        .shotWindowValid = true,
        .shotWindowCenter = clock.time * 1000 + 150'000,
        .shotWindowHalfWidth = 100'000,
    };
    EXPECT_CALL(ballistics, computeTurretAimAngles).WillOnce(Return(solution));

    AutoAimLaunchTimer timer(0, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::GATED_DENY, result);
}

TEST_F(AutoAimLaunchTimerTest, pulse_estimation_after_window_denies_fire)
{
    ClockStub clock;
    clock.time = 700;  // 700ms

    VisionCoprocessor::TurretAimData aimData = {};
    aimData.targetState.updated = 1;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    CvBallisticsSolver::BallisticsSolution solution{
        .pitchAngle = 0,
        .yawAngle = 0,
        .yawVel = 0,
        .yawAcc = 0,
        .distance = 5.0f,
        .timeOfFlight = 0.2f,
        .activePlateIndex = 1,
        .shotWindowValid = true,
        .shotWindowCenter = clock.time * 1000 - 150'000,
        .shotWindowHalfWidth = 100'000,
    };
    EXPECT_CALL(ballistics, computeTurretAimAngles).WillOnce(Return(solution));

    AutoAimLaunchTimer timer(0, &visionCoprocessor, &ballistics);
    auto result = timer.getCurrentLaunchInclination(0);

    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::GATED_DENY, result);
}

TEST_F(AutoAimLaunchTimerTest, pulse_estimation_with_agitator_delay_within_window_allows_fire)
{
    ClockStub clock;
    clock.time = 400;  // 400ms

    VisionCoprocessor::TurretAimData aimData = {};
    aimData.targetState.updated = 1;

    EXPECT_CALL(visionCoprocessor, getLastAimData(0)).WillOnce(ReturnPointee(&aimData));

    CvBallisticsSolver::BallisticsSolution solution{
        .pitchAngle = 0,
        .yawAngle = 0,
        .yawVel = 0,
        .yawAcc = 0,
        .distance = 5.0f,
        .timeOfFlight = 0.2f,
        .activePlateIndex = 1,
        .shotWindowValid = true,
        .shotWindowCenter = clock.time * 1000 + 150'000,  // Accounting for agitator delay
        .shotWindowHalfWidth = 100'000,
    };
    EXPECT_CALL(ballistics, computeTurretAimAngles).WillOnce(Return(solution));

    AutoAimLaunchTimer timer(50'000, &visionCoprocessor, &ballistics);  // 50ms agitator delay
    auto result = timer.getCurrentLaunchInclination(0);

    // effectiveFireTime = 400ms + 50ms = 450ms, which is at shotWindowStart
    ASSERT_EQ(AutoAimLaunchTimer::LaunchInclination::GATED_ALLOW, result);
}
