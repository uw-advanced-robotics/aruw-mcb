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

#include <gtest/gtest.h>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/drivers.hpp"

// #include "aruwsrc/robot/engineer/algorithms/inverse_kinematics/abstract_ik_command.hpp"
#include "aruwsrc/robot/engineer/algorithms/inverse_kinematics/trajectory_6d.hpp"

using namespace testing;
using namespace tap::algorithms::transforms;
using namespace aruwsrc::engineer::algorithms::inverse_kinematics;

const float EPS = 1E-5;

inline void expectEq(const Position& actual, const Position& expected, const float epsilon = EPS)
{
    EXPECT_NEAR(actual.x(), expected.x(), epsilon);
    EXPECT_NEAR(actual.y(), expected.y(), epsilon);
    EXPECT_NEAR(actual.z(), expected.z(), epsilon);
}

inline void expectEq(const Vector& actual, const Vector& expected, const float epsilon = EPS)
{
    EXPECT_NEAR(actual.x(), expected.x(), epsilon);
    EXPECT_NEAR(actual.y(), expected.y(), epsilon);
    EXPECT_NEAR(actual.z(), expected.z(), epsilon);
}

inline void expectEq(
    const DynamicPosition& actual,
    const DynamicPosition& expected,
    const float epsilon = EPS)
{
    expectEq(actual.getPosition(), expected.getPosition(), epsilon);
    expectEq(actual.getVelocity(), expected.getVelocity(), epsilon);
    expectEq(actual.getAcceleration(), expected.getAcceleration(), epsilon);
}

inline void expectEq(
    const Orientation& actual,
    const Orientation& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.roll(), expected.roll(), epsilon);
    EXPECT_NEAR(actual.pitch(), expected.pitch(), epsilon);
    EXPECT_NEAR(actual.yaw(), expected.yaw(), epsilon);
}

inline void expectEq(
    const AngularVelocity& actual,
    const AngularVelocity& expected,
    const float epsilon = EPS)
{
    EXPECT_NEAR(actual.getRollVelocity(), expected.getRollVelocity(), epsilon);
    EXPECT_NEAR(actual.getPitchVelocity(), expected.getPitchVelocity(), epsilon);
    EXPECT_NEAR(actual.getYawVelocity(), expected.getYawVelocity(), epsilon);
}

inline void expectEq(
    const DynamicOrientation& actual,
    const DynamicOrientation& expected,
    const float epsilon = EPS)
{
    expectEq(actual.getRotation(), expected.getRotation(), epsilon);
    expectEq(actual.getAngularVelocity(), expected.getAngularVelocity(), epsilon);
}

inline void expectStaticEq(
    const Transform& actual,
    const Transform& expected,
    const float epsilon = EPS)
{
    expectEq(actual.getTranslation(), expected.getTranslation(), epsilon);
    expectEq(actual.getRotation(), expected.getRotation(), epsilon);
}

inline void expectEq(const Transform& actual, const Transform& expected, const float epsilon = EPS)
{
    expectEq(actual.getDynamicTranslation(), expected.getDynamicTranslation(), epsilon);
    expectEq(actual.getDynamicOrientation(), expected.getDynamicOrientation(), epsilon);
}

TEST(Trajectory6d, two_waypoint_identical)
{
    Trajectory6D<2> t{
        {{{
              .pose = tap::algorithms::transforms::Transform(),
              .time = 0.0f,
          },
          {
              .pose = tap::algorithms::transforms::Transform(),
              .time = 1.0f,
          }}}};

    for (float i = 0; i <= 1; i += 0.1)
    {
        Transform expected = tap::algorithms::transforms::Transform();
        expectEq(t.atTime(i), expected);
    }
}

TEST(Trajectory6d, two_waypoint_translation_only)
{
    Trajectory6D<2> t{
        {{{
              .pose = tap::algorithms::transforms::Transform(),
              .time = 0.0f,
          },
          {
              .pose = tap::algorithms::transforms::Transform(1, 1, 1, 0, 0, 0),
              .time = 1.0f,
          }}}};

    for (float i = 0; i <= 1; i += 0.1)
    {
        Transform expected = tap::algorithms::transforms::Transform(i, i, i, 0, 0, 0);
        expectEq(t.atTime(i), expected);
    }
}

TEST(Trajectory6d, two_waypoint_yaw_only)
{
    Trajectory6D<2> t{
        {{{
              .pose = tap::algorithms::transforms::Transform(),
              .time = 0.0f,
          },
          {
              .pose = tap::algorithms::transforms::Transform(0, 0, 0, 0, 0, 3),
              .time = 1.0f,
          }}}};

    for (float i = 0; i <= 1; i += 0.1)
    {
        Transform expected = tap::algorithms::transforms::Transform(0, 0, 0, 0, 0, i * 3);
        expectEq(t.atTime(i), expected);
    }
}

TEST(Trajectory6d, two_waypoint_pitch_only)
{
    Trajectory6D<2> t{
        {{{
              .pose = tap::algorithms::transforms::Transform(0, 0, 0, 0, -1.5, 0),
              .time = 0.0f,
          },
          {
              .pose = tap::algorithms::transforms::Transform(0, 0, 0, 0, 1.5, 0),
              .time = 1.0f,
          }}}};

    for (float i = 0.1; i < 1 - 1e-5; i += 0.1)
    {
        Transform expected = tap::algorithms::transforms::Transform(0, 0, 0, 0, i * 3 - 1.5, 0);
        expectEq(t.atTime(i), expected);
    }
}

TEST(Trajectory6d, two_waypoint_roll_only)
{
    Trajectory6D<2> t{
        {{{
              .pose = tap::algorithms::transforms::Transform(),
              .time = 0.0f,
          },
          {
              .pose = tap::algorithms::transforms::Transform(0, 0, 0, 3, 0, 0),
              .time = 1.0f,
          }}}};

    for (float i = 0; i <= 1; i += 0.1)
    {
        Transform expected = tap::algorithms::transforms::Transform(0, 0, 0, i * 3, 0, 0);
        expectEq(t.atTime(i), expected);
    }
}