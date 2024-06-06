/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/algorithms/auto_nav_path.hpp"

using namespace testing;
using namespace aruwsrc::algorithms;

class AutoNavPathTest : public Test
{
protected:
    constexpr static float kInterpolationDistance = 2.5f;  // number of segments between path points

    AutoNavPathTest() : path() {}

    AutoNavPath path;
};

TEST_F(AutoNavPathTest, getClosestOnSegment_clamp_to_first_point)
{
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(-1.0f, -1.0f, 0.0f);
    float closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(0.0f, closest);

    // projection is exactly on p1
    current = Position(0.0f, -5.5f, 0.0f);
    closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(0.0f, closest);

    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(6.0f, -1.0f, 0.0f);
    closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(0.0f, closest);
}

TEST_F(AutoNavPathTest, getClosestOnSegment_clamp_to_second_point)
{
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(6.0f, -1.0f, 0.0f);
    float closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(5.0f, closest);

    // projection is exactly on p2
    current = Position(5.0f, -5.5f, 0.0f);
    closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(5.0f, closest);

    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(-1.0f, -1.0f, 0.0f);
    closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(5.0f, closest);
}

TEST_F(AutoNavPathTest, getClosestOnSegment_projected_on_segment)
{
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(2.5f, -1.0f, 0.0f);
    float closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(2.5f, closest);

    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(2.5f, -1.0f, 0.0f);
    closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(2.5f, closest);
}
