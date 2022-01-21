/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp"

using namespace aruwsrc::control::turret::algorithms;

TEST(TurretGravityCompensation, computeGravitationalForceOffset_zero_cg_offsets_return_0)
{
    EXPECT_FLOAT_EQ(0, computeGravitationalForceOffset(0, 0, 0, 0));
    EXPECT_FLOAT_EQ(0, computeGravitationalForceOffset(0, 0, 0, 100));
    EXPECT_FLOAT_EQ(0, computeGravitationalForceOffset(0, 0, 10, 100));
    EXPECT_FLOAT_EQ(0, computeGravitationalForceOffset(0, 0, 30, 100));
}

TEST(
    TurretGravityCompensation,
    computeGravitationalForceOffset_zero_gravityCompensationMax_returns_0)
{
    EXPECT_FLOAT_EQ(0, computeGravitationalForceOffset(10, 0, 0, 0));
    EXPECT_FLOAT_EQ(0, computeGravitationalForceOffset(10, 10, 0, 0));
    EXPECT_FLOAT_EQ(0, computeGravitationalForceOffset(10, 10, 10, 0));
}

TEST(
    TurretGravityCompensation,
    computeGravitationalForceOffset_0deg_cg_offset_returns_offset_as_function_of_pitch_offset)
{
    std::vector<float> anglesToTest{0, 45, 90, 135, 180, -135, -90, -45};

    for (float angle : anglesToTest)
    {
        EXPECT_NEAR(
            100 * cos(modm::toRadian(angle)),
            computeGravitationalForceOffset(10, 0, angle, 100),
            1E-3);
    }
}

TEST(TurretGravityCompensation, computeGravitationalForceOffset_45deg_cg_offset)
{
    EXPECT_NEAR(
        100 * cos(modm::toRadian(45)),
        computeGravitationalForceOffset(10, 10, 0, 100),
        1E-3);
}

TEST(TurretGravityCompensation, computeGravitationalForceOffset_90deg_cg_offset)
{
    EXPECT_NEAR(0, computeGravitationalForceOffset(0, 10, 0, 100), 1E-3);
}

TEST(TurretGravityCompensation, computeGravitationalForceOffset_135deg_cg_offset)
{
    EXPECT_NEAR(
        100 * cos(modm::toRadian(135)),
        computeGravitationalForceOffset(-10, 10, 0, 100),
        1E-3);
}

TEST(TurretGravityCompensation, computeGravitationalForceOffset_180deg_cg_offset)
{
    EXPECT_NEAR(
        100 * cos(modm::toRadian(180)),
        computeGravitationalForceOffset(-10, 0, 0, 100),
        1E-3);
}

TEST(TurretGravityCompensation, computeGravitationalForceOffset_225deg_cg_offset)
{
    EXPECT_NEAR(
        100 * cos(modm::toRadian(225)),
        computeGravitationalForceOffset(-10, -10, 0, 100),
        1E-3);
}

TEST(TurretGravityCompensation, computeGravitationalForceOffset_270deg_cg_offset)
{
    EXPECT_NEAR(
        100 * cos(modm::toRadian(270)),
        computeGravitationalForceOffset(0, -10, 0, 100),
        1E-3);
}

TEST(TurretGravityCompensation, computeGravitationalForceOffset_315deg_cg_offset)
{
    EXPECT_NEAR(
        100 * cos(modm::toRadian(315)),
        computeGravitationalForceOffset(10, -10, 0, 100),
        1E-3);
}
