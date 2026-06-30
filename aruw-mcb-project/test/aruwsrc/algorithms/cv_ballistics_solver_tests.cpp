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
#include "tap/mock/odometry_2d_interface_mock.hpp"

#include "aruwsrc/algorithms/ballistics/cv_ballistics_solver.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/mock/launch_speed_predictor_interface_mock.hpp"
#include "aruwsrc/mock/transformer_interface_mock.hpp"
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"

using namespace testing;
using namespace aruwsrc::algorithms::ballistics;

CvBallisticsSolver::Config BALLISTICS_CONFIG{
    .shotTimingEntryThreshold = 6.0f,
    .shotTimingExitThreshold = 4.0f,
    .defaultLaunchSpeed = 15,
    .turretPitchOffset = 0,
    .minimumShotDelay = 0.0f,
};

struct WithinAimingToleranceConfig
{
    bool withinTolerance = 0;
    float yawAngleError = 0;
    float pitchAngleError = 0;
    float targetDistance = 0;
    char padding[3] = {};
} modm_packed;

class WithinAimingToleranceTest : public TestWithParam<WithinAimingToleranceConfig>
{
};

TEST_P(WithinAimingToleranceTest, various_values)
{
    EXPECT_EQ(
        GetParam().withinTolerance,
        CvBallisticsSolver::withinAimingTolerance(
            GetParam().yawAngleError,
            GetParam().pitchAngleError,
            GetParam().targetDistance));
}

std::vector<WithinAimingToleranceConfig> withinAimingToleranceValuesToTest = {
    {
        .withinTolerance = false,
        .yawAngleError = 0,
        .pitchAngleError = 0,
        .targetDistance = -10,
    },
    {
        .withinTolerance = true,
        .yawAngleError = 0,
        .pitchAngleError = 0,
        .targetDistance = 1,
    },
    {
        .withinTolerance = true,
        .yawAngleError = modm::toRadian(1),     ///< Very small angle error
        .pitchAngleError = -modm::toRadian(1),  ///< Very small angle error
        .targetDistance = 1,
    },
    {
        .withinTolerance = true,
        .yawAngleError = -modm::toRadian(1),   ///< Very small angle error
        .pitchAngleError = modm::toRadian(1),  ///< Very small angle error
        .targetDistance = 1,
    },
    {
        .withinTolerance = false,
        .yawAngleError = modm::toRadian(30),     ///< Very large angle error
        .pitchAngleError = -modm::toRadian(30),  ///< Very large angle error
        .targetDistance = 1,
    },
    {
        .withinTolerance = false,
        .yawAngleError = -modm::toRadian(30),   ///< Very large angle error
        .pitchAngleError = modm::toRadian(30),  ///< Very large angle error
        .targetDistance = 1,
    },
};

INSTANTIATE_TEST_SUITE_P(
    CvBallisticsSolver,
    WithinAimingToleranceTest,
    ValuesIn(withinAimingToleranceValuesToTest));

class CvBallisticsSolverTest : public Test
{
protected:
    CvBallisticsSolverTest()
        : vc(&drivers),
          worldToTurretYaw(0, 0, 0, 0, 0, 0),
          solver(
              // hack to set up default return transformer return value before ballistics
              // constructor uses it
              [this]() -> auto& {
                  ON_CALL(transformer, getWorldToTurretYaw)
                      .WillByDefault(testing::ReturnRef(worldToTurretYaw));
                  return vc;
              }(),
              transformer,
              launcher,
              BALLISTICS_CONFIG,
              0)
    {
    }

    void SetUp() override
    {
        ON_CALL(vc, isCvOnline).WillByDefault(ReturnPointee(&cvOnline));

        ON_CALL(vc, getLastAimData).WillByDefault(ReturnRef(aimData));

        ON_CALL(transformer, getLastComputedOdometryTime)
            .WillByDefault(ReturnPointee(&lastComputedOdomTime));

        ON_CALL(launcher, getPredictedLaunchSpeed).WillByDefault(ReturnPointee(&launchSpeed));
    }

    tap::Drivers drivers;

    NiceMock<aruwsrc::mock::VisionCoprocessorMock> vc;
    NiceMock<aruwsrc::mock::LaunchSpeedPredictorInterfaceMock> launcher;
    tap::algorithms::transforms::Transform worldToTurretYaw;
    NiceMock<aruwsrc::mock::TransformerInterfaceMock> transformer;

    CvBallisticsSolver solver;

    std::optional<CvBallisticsSolver::BallisticsSolution> solution;

    aruwsrc::communication::serial::VisionCoprocessor::TurretAimData aimData = {};
    uint32_t lastComputedOdomTime = 0;
    float launchSpeed = 15;
    bool cvOnline = true;
    tap::arch::clock::ClockStub clock;
};

TEST_F(CvBallisticsSolverTest, computeTurretAimAngles_cv_offline)
{
    cvOnline = false;

    solution = solver.computeTurretAimAngles();

    EXPECT_FALSE(solution.has_value());
}

TEST_F(CvBallisticsSolverTest, computeTurretAimAngles_aim_data_invalid)
{
    solution = solver.computeTurretAimAngles();

    aimData.targetState.updated = false;

    EXPECT_FALSE(solution.has_value());
}

TEST_F(CvBallisticsSolverTest, computeTurretAimAngles_timestamps_not_new)
{
    aimData.targetState.xPos = 2;

    solution = solver.computeTurretAimAngles();

    // if timestamps had changed solution would be valid
    EXPECT_FALSE(solution.has_value());
}

TEST_F(CvBallisticsSolverTest, computeTurretAimAngles_odom_timestamp_new)
{
    aimData.targetState.updated = true;
    aimData.targetState.xPos = 2;

    lastComputedOdomTime = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_NEAR(2, solution->distance, 1e-5);
}

TEST_F(CvBallisticsSolverTest, computeTurretAimAngles_aimData_timestamp_new)
{
    aimData.targetState.updated = true;
    aimData.targetState.xPos = 2;
    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_NEAR(2, solution->distance, 1e-5);
}

TEST_F(CvBallisticsSolverTest, computeTurretAimAngles_nonzero_robot_position)
{
    aimData.targetState.updated = true;
    aimData.targetState.xPos = 2;
    worldToTurretYaw.updateTranslation(-2, 0, 0);

    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_NEAR(4, solution->distance, 1e-5);
}

TEST_F(
    CvBallisticsSolverTest,
    computeTurretAimAngles_solution_found_no_new_time_solution_not_resolved)
{
    aimData.targetState.updated = true;
    aimData.targetState.xPos = 2;
    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_NEAR(2, solution->distance, 1e-5);

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_NEAR(2, solution->distance, 1e-5);
}

TEST_F(CvBallisticsSolverTest, computeTurretAimAngles_solution_found_no_valid_solution)
{
    aimData.targetState.updated = true;
    aimData.targetState.xPos = 100;
    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_FALSE(solution.has_value());
}

TEST_F(CvBallisticsSolverTest, jitter_aim_low_omega)
{
    aimData.targetState.updated = true;
    aimData.targetState.xPos = 2;
    aimData.targetState.yPos = 0;
    aimData.targetState.zPos = 0;
    aimData.targetState.omega = BALLISTICS_CONFIG.shotTimingExitThreshold - 1;
    aimData.targetState.radius0 = 0.2f;
    aimData.targetState.radius1 = 0.2f;
    aimData.targetState.theta = 0;
    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_FALSE(solution->shotWindowValid);
}

TEST_F(CvBallisticsSolverTest, pulse_estimation_high_omega)
{
    aimData.targetState.updated = true;
    aimData.targetState.xPos = 2;
    aimData.targetState.yPos = 0;
    aimData.targetState.zPos = 0;
    aimData.targetState.omega = BALLISTICS_CONFIG.shotTimingEntryThreshold + 1;
    aimData.targetState.radius0 = 0.2f;
    aimData.targetState.radius1 = 0.2f;
    aimData.targetState.theta = 0;
    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->shotWindowValid);
    EXPECT_NE(0, solution->shotWindowCenter);
    EXPECT_NE(0, solution->shotWindowHalfWidth);
    EXPECT_GE(solution->activePlateIndex, 0);
    EXPECT_LE(solution->activePlateIndex, 3);
}

TEST_F(CvBallisticsSolverTest, pulse_estimation_discards_when_omega_drops)
{
    aimData.targetState.updated = true;
    aimData.targetState.xPos = 2;
    aimData.targetState.yPos = 0;
    aimData.targetState.zPos = 0;
    aimData.targetState.omega = BALLISTICS_CONFIG.shotTimingEntryThreshold + 1;
    aimData.targetState.radius0 = 0.2f;
    aimData.targetState.radius1 = 0.2f;
    aimData.targetState.theta = 0;
    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->shotWindowValid);

    // Omega drops below threshold
    aimData.targetState.omega = BALLISTICS_CONFIG.shotTimingExitThreshold - 1;
    aimData.timestamp = 101;
    clock.time = 150;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());

    // Should have switched to jitter aim
    EXPECT_FALSE(solution->shotWindowValid);
}
