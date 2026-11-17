/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <cmath>

#include <gtest/gtest.h>

#include "aruwsrc/robot/engineer/wrist/wrist_subsystem.hpp"

using namespace tap::algorithms::transforms;
using aruwsrc::engineer::wrist::WristSubsystem;

static constexpr float EPS = 1e-4f;

static void expectSameOrientation(const Orientation& actual, const Orientation& expected)
{
    for (int i = 0; i < 9; i++)
    {
        EXPECT_NEAR(actual.matrix().data[i], expected.matrix().data[i], EPS);
    }
}

static void expectAngleNear(float actual, float expected)
{
    EXPECT_NEAR(tap::algorithms::Angle(actual).minDifference(expected), 0.0f, EPS);
}

TEST(WristSubsystem, getJointAnglesForOrientation_round_trips_positive_theta2)
{
    Orientation target = WristSubsystem::getHypotheticalOrientation(0.3f, 0.5f, -0.4f);

    auto [t1, t2, t3] = WristSubsystem::getJointAnglesForOrientation(target, 0.3f);

    expectAngleNear(t1, 0.3f);
    EXPECT_NEAR(t2, 0.5f, EPS);
    expectAngleNear(t3, -0.4f);
}

TEST(WristSubsystem, getJointAnglesForOrientation_round_trips_negative_theta2)
{
    Orientation target = WristSubsystem::getHypotheticalOrientation(0.3f, -0.5f, -0.4f);

    auto [t1, t2, t3] = WristSubsystem::getJointAnglesForOrientation(target, 0.3f);

    expectAngleNear(t1, 0.3f);
    EXPECT_NEAR(t2, -0.5f, EPS);
    expectAngleNear(t3, -0.4f);
}

TEST(WristSubsystem, getJointAnglesForOrientation_does_not_flip_when_theta2_crosses_zero)
{
    // theta1 is already right, so the solver should bend the other way instead of flipping theta1
    Orientation target = WristSubsystem::getHypotheticalOrientation(0.2f, -0.1f, 0.1f);

    auto [t1, t2, t3] = WristSubsystem::getJointAnglesForOrientation(target, 0.2f);

    expectAngleNear(t1, 0.2f);
    EXPECT_NEAR(t2, -0.1f, EPS);
    expectAngleNear(t3, 0.1f);
}

TEST(WristSubsystem, getJointAnglesForOrientation_holds_theta1_when_wrist_straight)
{
    Orientation target = WristSubsystem::getHypotheticalOrientation(0.3f, 0.0f, 0.2f);

    auto [t1, t2, t3] = WristSubsystem::getJointAnglesForOrientation(target, 0.1f);

    EXPECT_NEAR(t1, 0.1f, EPS);
    EXPECT_NEAR(t2, 0.0f, EPS);
    expectAngleNear(t3, 0.4f);  // theta1 + theta3 still 0.5
    expectSameOrientation(WristSubsystem::getHypotheticalOrientation(t1, t2, t3), target);
}

TEST(WristSubsystem, getJointAnglesForOrientation_limits_theta1_step)
{
    Orientation target = WristSubsystem::getHypotheticalOrientation(1.0f, 0.5f, 0.0f);

    auto [t1, t2, t3] = WristSubsystem::getJointAnglesForOrientation(target, 0.0f);

    EXPECT_NEAR(t1, WristSubsystem::MAX_THETA1_STEP, EPS);
    EXPECT_FALSE(std::isnan(t2));
    EXPECT_FALSE(std::isnan(t3));
}

TEST(WristSubsystem, getJointAnglesForOrientation_moves_theta1_toward_closest_solution_in_limit)
{
    // compare against checking every half turn shift of the solution
    for (float prev = -WristSubsystem::THETA1_LIMIT; prev <= WristSubsystem::THETA1_LIMIT;
         prev += 0.05f)
    {
        for (float direction = -M_PI; direction < M_PI; direction += 0.1f)
        {
            Orientation target = WristSubsystem::getHypotheticalOrientation(direction, 0.5f, 0.3f);
            const auto& r = target.matrix().data;
            float solution = atan2f(r[3], -r[6]);

            float closest = NAN;
            for (int halfTurns = -10; halfTurns <= 10; halfTurns++)
            {
                float candidate = solution + halfTurns * M_PI;
                if (std::abs(candidate) <= WristSubsystem::THETA1_LIMIT &&
                    (std::isnan(closest) || std::abs(candidate - prev) < std::abs(closest - prev)))
                {
                    closest = candidate;
                }
            }
            float expected = prev + std::clamp(
                                        closest - prev,
                                        -WristSubsystem::MAX_THETA1_STEP,
                                        WristSubsystem::MAX_THETA1_STEP);

            auto [t1, t2, t3] = WristSubsystem::getJointAnglesForOrientation(target, prev);
            EXPECT_NEAR(t1, expected, EPS) << "prev " << prev << " direction " << direction;
        }
    }
}

TEST(WristSubsystem, getJointAnglesForOrientation_does_not_wind_theta1)
{
    // bend direction circles around twice, which would wind theta1 two turns if it followed
    float theta1 = 0.0f;
    float minTheta1 = 0.0f, maxTheta1 = 0.0f;
    Orientation target;
    for (int i = 0; i <= 20000; i++)
    {
        float direction = 2 * 2 * M_PI * i / 20000.0f;
        target = WristSubsystem::getHypotheticalOrientation(direction, 0.5f, -direction);
        auto [t1, t2, t3] = WristSubsystem::getJointAnglesForOrientation(target, theta1);
        theta1 = t1;
        minTheta1 = std::min(minTheta1, theta1);
        maxTheta1 = std::max(maxTheta1, theta1);
    }

    EXPECT_GE(minTheta1, -WristSubsystem::THETA1_LIMIT - EPS);
    EXPECT_LE(maxTheta1, WristSubsystem::THETA1_LIMIT + EPS);

    // still reaches the orientation once it has time to settle
    float t2 = 0, t3 = 0;
    for (int i = 0; i < 1000; i++)
    {
        auto angles = WristSubsystem::getJointAnglesForOrientation(target, theta1);
        theta1 = angles[0];
        t2 = angles[1];
        t3 = angles[2];
    }
    expectSameOrientation(WristSubsystem::getHypotheticalOrientation(theta1, t2, t3), target);
}

TEST(WristSubsystem, getJointAnglesForOrientation_no_nan_when_float_error_exceeds_one)
{
    Orientation target(tap::algorithms::CMSISMat<3, 3>(
        {1.0000001f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}));

    auto [t1, t2, t3] = WristSubsystem::getJointAnglesForOrientation(target, 0.0f);

    EXPECT_FALSE(std::isnan(t1));
    EXPECT_FALSE(std::isnan(t2));
    EXPECT_FALSE(std::isnan(t3));
}

TEST(WristSubsystem, getJointAnglesForOrientation_round_trips_many_poses)
{
    for (float a1 = -3.0f; a1 <= 3.0f; a1 += 0.7f)
    {
        for (float a2 = -1.5f; a2 <= 1.5f; a2 += 0.3f)
        {
            for (float a3 = -3.0f; a3 <= 3.0f; a3 += 0.7f)
            {
                Orientation target = WristSubsystem::getHypotheticalOrientation(a1, a2, a3);
                auto [t1, t2, t3] = WristSubsystem::getJointAnglesForOrientation(target, a1);
                expectSameOrientation(
                    WristSubsystem::getHypotheticalOrientation(t1, t2, t3),
                    target);
            }
        }
    }
}
