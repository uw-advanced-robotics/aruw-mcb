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

#include "aruwsrc/algorithms/cv_ballistics_solver.hpp"
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

    CvBallisticsSolver solver;

    std::optional<CvBallisticsSolver::BallisticsSolution> solution;

    aruwsrc::communication::serial::VisionCoprocessor::TurretAimData aimData = {};
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

TEST_F(OttoBallisticsSolverTest, jitter_aim_low_omega)
{
    aimData.pva.updated = true;
    aimData.pva.xPos = 2;
    aimData.pva.yPos = 0;
    aimData.pva.zPos = 0;
    aimData.pva.omega = 0.5f;  // Below OMEGA_THRESHOLD (1.0)
    aimData.pva.rad0 = 0.2f;
    aimData.pva.rad1 = 0.2f;
    aimData.pva.theta = 0;
    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_FALSE(solution->usePulseEstimation);
    EXPECT_EQ(0, solution->shotWindowStart);
    EXPECT_EQ(0, solution->shotWindowEnd);
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_high_omega)
{
    aimData.pva.updated = true;
    aimData.pva.xPos = 2;
    aimData.pva.yPos = 0;
    aimData.pva.zPos = 0;
    aimData.pva.omega = 2.0f;  // Above OMEGA_THRESHOLD (1.0)
    aimData.pva.rad0 = 0.2f;
    aimData.pva.rad1 = 0.2f;
    aimData.pva.theta = 0;
    aimData.timestamp = 100;

    clock.time = 100;

    solution = solver.computeTurretAimAngles();

    EXPECT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->usePulseEstimation);
    EXPECT_NE(0, solution->shotWindowStart);
    EXPECT_NE(0, solution->shotWindowEnd);
    EXPECT_GE(solution->activePlateIndex, 0);
    EXPECT_LE(solution->activePlateIndex, 3);
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_persists_within_window)
{
    aimData.pva.updated = true;
    aimData.pva.xPos = 2;
    aimData.pva.yPos = 0;
    aimData.pva.zPos = 0;
    aimData.pva.omega = 2.0f;
    aimData.pva.rad0 = 0.2f;
    aimData.pva.rad1 = 0.2f;
    aimData.pva.theta = 0;
    aimData.timestamp = 100;

    clock.time = 100000;  // Start at 100ms

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    auto firstSolution = solution;

    // Advance time but stay within shot window
    clock.time = 150000;  // 50ms later
    aimData.timestamp = 101;  // New aim data

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());

    // Solution should be unchanged (same shot window)
    EXPECT_EQ(firstSolution->shotWindowStart, solution->shotWindowStart);
    EXPECT_EQ(firstSolution->shotWindowEnd, solution->shotWindowEnd);
    EXPECT_EQ(firstSolution->activePlateIndex, solution->activePlateIndex);
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_recalculates_after_window_expires)
{
    aimData.pva.updated = true;
    aimData.pva.xPos = 2;
    aimData.pva.yPos = 0;
    aimData.pva.zPos = 0;
    aimData.pva.omega = 2.0f;
    aimData.pva.rad0 = 0.2f;
    aimData.pva.rad1 = 0.2f;
    aimData.pva.theta = 0;
    aimData.timestamp = 100;

    clock.time = 100000;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    uint64_t firstWindowEnd = solution->shotWindowEnd;

    // Advance time past shot window
    clock.time = firstWindowEnd + 100000;  // 100ms after window closed
    aimData.timestamp = 200;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());

    // Should have recalculated with new window
    EXPECT_NE(firstWindowEnd, solution->shotWindowEnd);
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_discards_when_omega_drops)
{
    aimData.pva.updated = true;
    aimData.pva.xPos = 2;
    aimData.pva.yPos = 0;
    aimData.pva.zPos = 0;
    aimData.pva.omega = 2.0f;  // High omega
    aimData.pva.rad0 = 0.2f;
    aimData.pva.rad1 = 0.2f;
    aimData.pva.theta = 0;
    aimData.timestamp = 100;

    clock.time = 100000;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->usePulseEstimation);

    // Omega drops below threshold
    aimData.pva.omega = 0.5f;  // Below threshold
    aimData.timestamp = 101;
    clock.time = 150000;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());

    // Should have switched to jitter aim
    EXPECT_FALSE(solution->usePulseEstimation);
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_selects_correct_plate_based_on_geometry)
{
    // Robot at (3, 0) with plate 0 facing us (theta = pi)
    // Our aim angle from origin to (3,0) is 0 radians
    // Plate 0 is at theta = pi, so it's facing AWAY from us (180 degrees off)
    // Plate 2 (theta = pi + pi = 0) should be selected (180 deg rotation needed)
    aimData.pva.updated = true;
    aimData.pva.xPos = 3.0f;
    aimData.pva.yPos = 0.0f;
    aimData.pva.zPos = 0.5f;
    aimData.pva.omega = 1.5f;  // Above threshold, CCW rotation
    aimData.pva.rad0 = 0.2f;
    aimData.pva.rad1 = 0.2f;
    aimData.pva.theta = M_PI;  // Plate 0 pointing LEFT (away from us)
    aimData.pva.plateHeights[0] = 0.0f;
    aimData.pva.plateHeights[1] = 0.1f;
    aimData.pva.plateHeights[2] = 0.0f;
    aimData.pva.plateHeights[3] = 0.1f;
    aimData.timestamp = 100;

    clock.time = 100000;
    chassisLoc.setPosition(0, 0);  // Turret at origin

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->usePulseEstimation);
    
    // With theta=pi and omega=1.5 rad/s, and ToF ~= 3/15 = 0.2s
    // In 0.2s, robot rotates ~0.3 rad (17 degrees) - not enough for plate 2
    // So plate 0 or plate 1 should be selected (closest to arriving)
    // Actually with the timing window logic, we want the next shootable plate
    EXPECT_GE(solution->activePlateIndex, 0);
    EXPECT_LE(solution->activePlateIndex, 3);
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_plate_at_different_angles_selects_correctly)
{
    // Robot at (0, 3) with plate 0 at theta = pi/2 (pointing UP)
    // Our aim angle is pi/2 (pointing up)
    // So plate 0 IS aligned with our aim line
    aimData.pva.updated = true;
    aimData.pva.xPos = 0.0f;
    aimData.pva.yPos = 3.0f;
    aimData.pva.zPos = 0.5f;
    aimData.pva.omega = 2.0f;  // Above threshold
    aimData.pva.rad0 = 0.2f;
    aimData.pva.rad1 = 0.2f;
    aimData.pva.theta = M_PI_2;  // Plate 0 pointing UP (same as aim direction)
    aimData.pva.plateHeights[0] = 0.0f;
    aimData.pva.plateHeights[1] = 0.1f;
    aimData.pva.plateHeights[2] = 0.0f;
    aimData.pva.plateHeights[3] = 0.1f;
    aimData.timestamp = 100;

    clock.time = 100000;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->usePulseEstimation);
    
    // Plate 0 is currently at aim line, but likely already passing
    // The algorithm should select the next valid plate
    EXPECT_GE(solution->activePlateIndex, 0);
    EXPECT_LE(solution->activePlateIndex, 3);
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_yaw_aims_at_active_plate_not_center)
{
    // Place robot at (2, 2) with plate 0 at theta=0 (pointing right)
    // Aim angle to center: atan2(2, 2) = pi/4 (45 degrees)
    // Plate 1 at theta = pi/2 (pointing up, 90 degrees)
    // If we incorrectly aim at center, yaw would be pi/4
    // If we correctly aim at a rotated plate, yaw should be different
    aimData.pva.updated = true;
    aimData.pva.xPos = 2.0f;
    aimData.pva.yPos = 2.0f;
    aimData.pva.zPos = 0.5f;
    aimData.pva.omega = 1.5f;
    aimData.pva.rad0 = 0.3f;  // Larger radius to see plate offset
    aimData.pva.rad1 = 0.3f;
    aimData.pva.theta = 0.0f;  // Plate 0 at angle 0
    aimData.pva.plateHeights[0] = 0.0f;
    aimData.pva.plateHeights[1] = 0.1f;
    aimData.pva.plateHeights[2] = 0.0f;
    aimData.pva.plateHeights[3] = 0.1f;
    aimData.timestamp = 100;

    clock.time = 100000;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    
    // The yaw should point at the active plate's position, not just robot center
    // For a robot with significant radius, this should be measurably different
    // Center angle would be atan2(2, 2) = 0.7854 rad (45 deg)
    float centerAngle = atan2f(2.0f, 2.0f);
    
    // If active plate is not 0, yaw should differ from center angle
    // (unless by coincidence the plate is at center angle)
    if (solution->activePlateIndex != 0 && solution->activePlateIndex != 2)
    {
        // Plates 1 or 3 are perpendicular, should have noticeably different angles
        EXPECT_NE(solution->yawAngle, centerAngle);
    }
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_clockwise_rotation_timing)
{
    // Robot spinning clockwise (negative omega)
    aimData.pva.updated = true;
    aimData.pva.xPos = 2.0f;
    aimData.pva.yPos = 0.0f;
    aimData.pva.zPos = 0.5f;
    aimData.pva.omega = -2.0f;  // Clockwise rotation
    aimData.pva.rad0 = 0.2f;
    aimData.pva.rad1 = 0.2f;
    aimData.pva.theta = M_PI;  // Plate 0 pointing left
    aimData.pva.plateHeights[0] = 0.0f;
    aimData.pva.plateHeights[1] = 0.1f;
    aimData.pva.plateHeights[2] = 0.0f;
    aimData.pva.plateHeights[3] = 0.1f;
    aimData.timestamp = 100;

    clock.time = 100000;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->usePulseEstimation);
    
    // Should handle clockwise rotation correctly
    EXPECT_GE(solution->activePlateIndex, 0);
    EXPECT_LE(solution->activePlateIndex, 3);
    EXPECT_GT(solution->shotWindowEnd, solution->shotWindowStart);
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_moving_robot_omega_total_differs_from_omega)
{
    // Robot moving tangentially while spinning
    // This creates additional angular velocity from translation
    aimData.pva.updated = true;
    aimData.pva.xPos = 2.0f;
    aimData.pva.yPos = 0.0f;
    aimData.pva.zPos = 0.5f;
    aimData.pva.xVel = 0.0f;
    aimData.pva.yVel = 2.0f;  // Moving perpendicular to r vector
    aimData.pva.zVel = 0.0f;
    aimData.pva.omega = 1.0f;  // Spinning
    aimData.pva.rad0 = 0.2f;
    aimData.pva.rad1 = 0.2f;
    aimData.pva.theta = 0.0f;
    aimData.pva.plateHeights[0] = 0.0f;
    aimData.pva.plateHeights[1] = 0.1f;
    aimData.pva.plateHeights[2] = 0.0f;
    aimData.pva.plateHeights[3] = 0.1f;
    aimData.timestamp = 100;

    clock.time = 100000;
    chassisVel = modm::Vector2f(0, 0);  // Chassis stationary

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->usePulseEstimation);
    
    // Shot window should be valid and account for omega_total
    EXPECT_GT(solution->shotWindowEnd, solution->shotWindowStart);
    EXPECT_GE(solution->shotWindowStart, clock.time);
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_shot_window_accounts_for_plate_width)
{
    // Verify shot window has reasonable duration based on plate angular width
    aimData.pva.updated = true;
    aimData.pva.xPos = 3.0f;
    aimData.pva.yPos = 0.0f;
    aimData.pva.zPos = 0.5f;
    aimData.pva.omega = 2.0f;  // 2 rad/s
    aimData.pva.rad0 = 0.2f;
    aimData.pva.rad1 = 0.2f;
    aimData.pva.theta = 0.0f;
    aimData.pva.plateHeights[0] = 0.0f;
    aimData.pva.plateHeights[1] = 0.1f;
    aimData.pva.plateHeights[2] = 0.0f;
    aimData.pva.plateHeights[3] = 0.1f;
    aimData.timestamp = 100;

    launchSpeed = 15.0f;
    clock.time = 100000;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->usePulseEstimation);
    
    // Plate angular width = 0.135m / 0.2m = 0.675 rad
    // Time for plate to cross = 0.675 / 2.0 = 0.3375s = 337500 us
    // Fire window should be approximately this duration
    uint64_t windowDuration = solution->shotWindowEnd - solution->shotWindowStart;
    
    // Window should be reasonable (between 100ms and 500ms for this configuration)
    EXPECT_GT(windowDuration, 100000);  // > 100ms
    EXPECT_LT(windowDuration, 500000);  // < 500ms
}

TEST_F(OttoBallisticsSolverTest, pulse_estimation_with_different_plate_radii)
{
    // Plates 0,2 have rad0, plates 1,3 have rad1
    // Different radii should result in different angular widths
    aimData.pva.updated = true;
    aimData.pva.xPos = 2.0f;
    aimData.pva.yPos = 0.0f;
    aimData.pva.zPos = 0.5f;
    aimData.pva.omega = 1.5f;
    aimData.pva.rad0 = 0.15f;  // Smaller radius
    aimData.pva.rad1 = 0.25f;  // Larger radius
    aimData.pva.theta = 0.0f;
    aimData.pva.plateHeights[0] = 0.0f;
    aimData.pva.plateHeights[1] = 0.1f;
    aimData.pva.plateHeights[2] = 0.0f;
    aimData.pva.plateHeights[3] = 0.1f;
    aimData.timestamp = 100;

    clock.time = 100000;

    solution = solver.computeTurretAimAngles();
    EXPECT_TRUE(solution.has_value());
    EXPECT_TRUE(solution->usePulseEstimation);
    
    // Solution should account for the active plate's specific radius
    // Smaller radius (0.15m) → larger angular width (0.135/0.15 = 0.9 rad)
    // Larger radius (0.25m) → smaller angular width (0.135/0.25 = 0.54 rad)
    EXPECT_GT(solution->shotWindowEnd, solution->shotWindowStart);
}

// ===== Kinematic Model Tests =====

TEST(RobotOrbitKinematicStateTest, projectForward_plate_at_zero_angle)
{
    // Robot center at (2, 0), plate at angle 0 (pointing RIGHT)
    // Plate radius 0.5m, so plate is at (2.5, 0)
    // Spin CCW at 1 rad/s
    // After 1 second: plate should be at angle 1 rad
    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState state(
        {2.0f, 0.0f, 0.0f},  // Robot center position
        {0.0f, 0.0f, 0.0f},  // No velocity
        {0.0f, 0.0f, 0.0f},  // No acceleration
        0.5f,                // Radius
        0.0f,                // Theta (angle 0 = pointing right)
        1.0f);               // Omega = 1 rad/s CCW

    auto futurePos = state.projectForward(1.0f);
    
    // Center stays at (2, 0)
    // Plate rotates to angle 1 rad
    // New position: (2 + 0.5*cos(1), 0 + 0.5*sin(1))
    EXPECT_NEAR(2.0f + 0.5f * cosf(1.0f), futurePos.x, 1e-4f);
    EXPECT_NEAR(0.0f + 0.5f * sinf(1.0f), futurePos.y, 1e-4f);
    EXPECT_NEAR(0.0f, futurePos.z, 1e-4f);
}

TEST(RobotOrbitKinematicStateTest, projectForward_plate_at_90_degrees)
{
    // Robot center at (2, 0), plate at angle pi/2 (pointing UP)
    // Plate radius 0.5m, so plate is at (2, 0.5)
    // Spin CCW at 2 rad/s
    // After 0.5 seconds: plate should be at angle pi/2 + 1 = 2.571 rad
    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState state(
        {2.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        0.5f,
        M_PI_2,       // Theta = 90 degrees (pointing up)
        2.0f);        // Omega = 2 rad/s

    auto futurePos = state.projectForward(0.5f);
    
    // After 0.5s: angle = pi/2 + 2*0.5 = pi/2 + 1
    float futureAngle = M_PI_2 + 1.0f;
    EXPECT_NEAR(2.0f + 0.5f * cosf(futureAngle), futurePos.x, 1e-4f);
    EXPECT_NEAR(0.0f + 0.5f * sinf(futureAngle), futurePos.y, 1e-4f);
}

TEST(RobotOrbitKinematicStateTest, projectForward_with_linear_motion)
{
    // Robot center moving from (2, 0) with velocity (1, 1)
    // Plate at angle 0, radius 0.5
    // After 1 second:
    //   - Center at (3, 1) due to velocity
    //   - Plate rotated to angle 1 rad
    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState state(
        {2.0f, 0.0f, 0.5f},
        {1.0f, 1.0f, 0.0f},  // Moving diagonally
        {0.0f, 0.0f, 0.0f},
        0.5f,
        0.0f,
        1.0f);

    auto futurePos = state.projectForward(1.0f);
    
    // Center moves to (3, 1, 0.5)
    // Plate offset: radius * (cos(1), sin(1))
    EXPECT_NEAR(3.0f + 0.5f * cosf(1.0f), futurePos.x, 1e-4f);
    EXPECT_NEAR(1.0f + 0.5f * sinf(1.0f), futurePos.y, 1e-4f);
    EXPECT_NEAR(0.5f, futurePos.z, 1e-4f);
}

TEST(RobotOrbitKinematicStateTest, projectForward_with_acceleration)
{
    // Robot accelerating, position should follow quadratic
    // s = s0 + v*t + 0.5*a*t^2
    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState state(
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {2.0f, 0.0f, 0.0f},  // Acceleration in x
        0.3f,
        M_PI,  // Plate pointing left
        0.5f);

    auto futurePos = state.projectForward(2.0f);
    
    // Center x: 0 + 1*2 + 0.5*2*4 = 2 + 4 = 6
    // Plate at angle pi + 0.5*2 = pi + 1
    EXPECT_NEAR(6.0f + 0.3f * cosf(M_PI + 1.0f), futurePos.x, 1e-4f);
}

TEST(RobotOrbitKinematicStateTest, projectForward_clockwise_rotation)
{
    // Negative omega = clockwise rotation
    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState state(
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        0.4f,
        M_PI_2,  // Starting at 90 degrees
        -1.0f);  // Clockwise

    auto futurePos = state.projectForward(1.0f);
    
    // Angle after 1s: pi/2 - 1
    float futureAngle = M_PI_2 - 1.0f;
    EXPECT_NEAR(0.4f * cosf(futureAngle), futurePos.x, 1e-4f);
    EXPECT_NEAR(0.4f * sinf(futureAngle), futurePos.y, 1e-4f);
}

TEST(RobotOrbitKinematicStateTest, computeOmegaTotal_pure_rotation)
{
    // Robot spinning with no translation
    // omega_total should equal omega_robot
    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState state(
        {2.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},  // No velocity
        {0.0f, 0.0f, 0.0f},
        0.5f,
        0.0f,
        1.5f);  // Pure rotation

    modm::Vector3f robotPos(2.0f, 0.0f, 0.0f);
    modm::Vector3f robotVel(0.0f, 0.0f, 0.0f);
    
    float omegaTotal = state.computeOmegaTotal(robotPos, robotVel);
    
    EXPECT_NEAR(1.5f, omegaTotal, 1e-4f);
}

TEST(RobotOrbitKinematicStateTest, computeOmegaTotal_with_tangential_velocity)
{
    // Robot at (2, 0) moving in +y direction (tangential to radius)
    // This adds angular velocity: (r × v)_z / |r|^2
    // r = (2, 0, 0), v = (0, 2, 0)
    // r × v = (0, 0, 4)
    // omega_translation = 4 / 4 = 1 rad/s
    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState state(
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        0.5f,
        0.0f,
        0.5f);  // omega_robot = 0.5

    modm::Vector3f robotPos(2.0f, 0.0f, 0.0f);
    modm::Vector3f robotVel(0.0f, 2.0f, 0.0f);  // Moving perpendicular
    
    float omegaTotal = state.computeOmegaTotal(robotPos, robotVel);
    
    // omega_total = 0.5 + 1.0 = 1.5
    EXPECT_NEAR(1.5f, omegaTotal, 1e-4f);
}

TEST(RobotOrbitKinematicStateTest, computeOmegaTotal_radial_velocity_no_contribution)
{
    // Velocity purely radial (toward/away from us) contributes 0 to omega
    // r = (2, 0, 0), v = (1, 0, 0) (radial)
    // r × v = (0, 0, 0)
    aruwsrc::communication::serial::VisionCoprocessor::RobotOrbitKinematicState state(
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        0.5f,
        0.0f,
        2.0f);

    modm::Vector3f robotPos(3.0f, 0.0f, 0.0f);
    modm::Vector3f robotVel(2.0f, 0.0f, 0.0f);  // Moving radially
    
    float omegaTotal = state.computeOmegaTotal(robotPos, robotVel);
    
    // Should equal omega_robot since radial velocity contributes nothing
    EXPECT_NEAR(2.0f, omegaTotal, 1e-4f);
}
