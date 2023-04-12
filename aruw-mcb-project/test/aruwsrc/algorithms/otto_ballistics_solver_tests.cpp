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

#include "aruwsrc/algorithms/otto_ballistics_solver.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/mock/launch_speed_predictor_interface_mock.hpp"
#include "aruwsrc/mock/robot_turret_subsystem_mock.hpp"
#include "aruwsrc/mock/vision_coprocessor_mock.hpp"

using namespace testing;
using namespace aruwsrc::algorithms;

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
        OttoBallisticsSolver::withinAimingTolerance(
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
    OttoBallisticsSolver,
    WithinAimingToleranceTest,
    ValuesIn(withinAimingToleranceValuesToTest));

class OttoBallisticsSolverTest : public Test
{
protected:
    OttoBallisticsSolverTest()
        : vc(&drivers),
          turret(&drivers),
          solver(vc, odometry, turret, launcher, 15, 0)
    {
    }

    void SetUp() override
    {
        ON_CALL(vc, isCvOnline).WillByDefault(ReturnPointee(&cvOnline));

        ON_CALL(vc, getLastAimData).WillByDefault(ReturnRef(aimData));

        ON_CALL(odometry, getLastComputedOdometryTime)
            .WillByDefault(ReturnPointee(&lastComputedOdomTime));
        ON_CALL(odometry, getCurrentLocation2D).WillByDefault(ReturnPointee(&chassisLoc));
        ON_CALL(odometry, getCurrentVelocity2D).WillByDefault(ReturnPointee(&chassisVel));

        ON_CALL(launcher, getPredictedLaunchSpeed).WillByDefault(ReturnPointee(&launchSpeed));
    }

    tap::Drivers drivers;

    NiceMock<aruwsrc::mock::VisionCoprocessorMock> vc;
    NiceMock<tap::mock::Odometry2DInterfaceMock> odometry;
    NiceMock<aruwsrc::mock::LaunchSpeedPredictorInterfaceMock> launcher;
    NiceMock<aruwsrc::mock::RobotTurretSubsystemMock> turret;

    OttoBallisticsSolver solver;

    std::optional<OttoBallisticsSolver::BallisticsSolution> solution;

    aruwsrc::serial::VisionCoprocessor::TurretAimData aimData = {};
    uint32_t lastComputedOdomTime = 0;
    float launchSpeed = 15;
    bool cvOnline = true;
    modm::Location2D<float> chassisLoc;
    modm::Vector2f chassisVel;
    tap::arch::clock::ClockStub clock;
};

TEST_F(OttoBallisticsSolverTest, computeTurretAimAngles_cv_offline)
{
    cvOnline = false;

    solution = solver.computeTurretAimAngles();

    EXPECT_FALSE(solution.has_value());
}

TEST_F(OttoBallisticsSolverTest, computeTurretAimAngles_aim_data_invalid)
{
    solution = solver.computeTurretAimAngles();

    aimData.pva.updated = false;

    EXPECT_FALSE(solution.has_value());
}

TEST_F(OttoBallisticsSolverTest, computeTurretAimAngles_timestamps_not_new)
{
    aimData.pva.xPos = 2;

    solution = solver.computeTurretAimAngles();

    // if timestamps had changed solution would be valid
    EXPECT_FALSE(solution.has_value());
}

TEST_F(OttoBallisticsSolverTest, computeTurretAimAngles_odom_timestamp_new)
{
    aimData.pva.updated = true;
    aimData.pva.xPos = 2;

    lastComputedOdomTime = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_NEAR(2, solution->distance, 1e-5);
}

TEST_F(OttoBallisticsSolverTest, computeTurretAimAngles_aimData_timestamp_new)
{
    aimData.pva.updated = true;
    aimData.pva.xPos = 2;
    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_NEAR(2, solution->distance, 1e-5);
}

TEST_F(OttoBallisticsSolverTest, computeTurretAimAngles_nonzero_robot_position)
{
    aimData.pva.updated = true;
    aimData.pva.xPos = 2;
    chassisLoc.setPosition(-2, 0);

    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_NEAR(4, solution->distance, 1e-5);
}

TEST_F(
    OttoBallisticsSolverTest,
    comiputeTurretAimAngles_solution_found_no_new_time_solution_not_resolved)
{
    aimData.pva.updated = true;
    aimData.pva.xPos = 2;
    aimData.timestamp = 100;

    clock.time = 100;

    EXPECT_CALL(odometry, getCurrentLocation2D).Times(1);

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_NEAR(2, solution->distance, 1e-5);

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_NEAR(2, solution->distance, 1e-5);
}

TEST_F(OttoBallisticsSolverTest, comiputeTurretAimAngles_solution_found_no_valid_solution)
{
    aimData.pva.updated = true;
    aimData.pva.xPos = 100;
    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_FALSE(solution.has_value());
}
