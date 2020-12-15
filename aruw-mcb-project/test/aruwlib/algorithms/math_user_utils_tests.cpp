/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <gtest/gtest.h>

using namespace aruwlib::algorithms;

TEST(MathUserUtils, copyRowColElimTwoByTwo)
{
    const float aArr[] = {-7, -4, 4, 1};
    modm::Matrix2f a(aArr);
    modm::Matrix1f out;

    EXPECT_TRUE(copyRowColElim(a, &out, 0, 0));
    EXPECT_FLOAT_EQ(1, out[0][0]);
}

TEST(MathUserUtils, copyRowColElimThreeByThree)
{
    const float aArr[] = {-7, -4, 4, 1, 1, 6, 5, -40, 4};
    modm::Matrix3f a(aArr);
    modm::Matrix2f out;

    EXPECT_TRUE(copyRowColElim(a, &out, 0, 0));
    EXPECT_FLOAT_EQ(1, out[0][0]);
    EXPECT_FLOAT_EQ(6, out[0][1]);
    EXPECT_FLOAT_EQ(-40, out[1][0]);
    EXPECT_FLOAT_EQ(4, out[1][1]);

    EXPECT_TRUE(copyRowColElim(a, &out, 0, 1));
    EXPECT_FLOAT_EQ(1, out[0][0]);
    EXPECT_FLOAT_EQ(6, out[0][1]);
    EXPECT_FLOAT_EQ(5, out[1][0]);
    EXPECT_FLOAT_EQ(4, out[1][1]);

    EXPECT_TRUE(copyRowColElim(a, &out, 0, 2));
    EXPECT_FLOAT_EQ(1, out[0][0]);
    EXPECT_FLOAT_EQ(1, out[0][1]);
    EXPECT_FLOAT_EQ(5, out[1][0]);
    EXPECT_FLOAT_EQ(-40, out[1][1]);

    EXPECT_TRUE(copyRowColElim(a, &out, 1, 0));
    EXPECT_FLOAT_EQ(-4, out[0][0]);
    EXPECT_FLOAT_EQ(4, out[0][1]);
    EXPECT_FLOAT_EQ(-40, out[1][0]);
    EXPECT_FLOAT_EQ(4, out[1][1]);

    EXPECT_TRUE(copyRowColElim(a, &out, 1, 1));
    EXPECT_FLOAT_EQ(-7, out[0][0]);
    EXPECT_FLOAT_EQ(4, out[0][1]);
    EXPECT_FLOAT_EQ(5, out[1][0]);
    EXPECT_FLOAT_EQ(4, out[1][1]);

    EXPECT_TRUE(copyRowColElim(a, &out, 1, 2));
    EXPECT_FLOAT_EQ(-7, out[0][0]);
    EXPECT_FLOAT_EQ(-4, out[0][1]);
    EXPECT_FLOAT_EQ(5, out[1][0]);
    EXPECT_FLOAT_EQ(-40, out[1][1]);

    EXPECT_TRUE(copyRowColElim(a, &out, 2, 0));
    EXPECT_FLOAT_EQ(-4, out[0][0]);
    EXPECT_FLOAT_EQ(4, out[0][1]);
    EXPECT_FLOAT_EQ(1, out[1][0]);
    EXPECT_FLOAT_EQ(6, out[1][1]);

    EXPECT_TRUE(copyRowColElim(a, &out, 2, 1));
    EXPECT_FLOAT_EQ(-7, out[0][0]);
    EXPECT_FLOAT_EQ(4, out[0][1]);
    EXPECT_FLOAT_EQ(1, out[1][0]);
    EXPECT_FLOAT_EQ(6, out[1][1]);

    EXPECT_TRUE(copyRowColElim(a, &out, 2, 2));
    EXPECT_FLOAT_EQ(-7, out[0][0]);
    EXPECT_FLOAT_EQ(-4, out[0][1]);
    EXPECT_FLOAT_EQ(1, out[1][0]);
    EXPECT_FLOAT_EQ(1, out[1][1]);
}

TEST(MathUserUtils, adjointOneByOne)
{
    const float aArr[] = {1};
    modm::Matrix1f a(aArr);
    modm::Matrix1f out;

    EXPECT_TRUE(adjoint(a, &out));
    EXPECT_FLOAT_EQ(1, out[0][0]);
}

// https://www.wolframalpha.com/input/?i=adjoint+%7B%7B1%2C2%7D%2C%7B3%2C4%7D%7D&assumption=%7B%22C%22%2C+%22adjoint%22%7D+-%3E+%7B%22LinearAlgebraWord%22%7D
TEST(MathUserUtils, adjointTwoByTwo)
{
    const float aArr[] = {1, 2, 3, 4};
    modm::Matrix2f a(aArr);
    modm::Matrix2f out;

    EXPECT_TRUE(adjoint(a, &out));
    EXPECT_FLOAT_EQ(4, out[0][0]);
    EXPECT_FLOAT_EQ(-2, out[0][1]);
    EXPECT_FLOAT_EQ(-3, out[1][0]);
    EXPECT_FLOAT_EQ(1, out[1][1]);
}

// https://www.wolframalpha.com/input/?i=adjoint+%7B%7B1%2C2%2C3%7D%2C%7B4%2C5%2C6%7D%2C%7B7%2C8%2C9%7D%7D&assumption=%7B%22C%22%2C+%22adjoint%22%7D+-%3E+%7B%22LinearAlgebraWord%22%7D
TEST(MathUserUtils, adjointThreeByThree)
{
    const float aArr[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    modm::Matrix3f a(aArr);
    modm::Matrix3f out;

    EXPECT_TRUE(adjoint(a, &out));
    EXPECT_FLOAT_EQ(-3, out[0][0]);
    EXPECT_FLOAT_EQ(6, out[0][1]);
    EXPECT_FLOAT_EQ(-3, out[0][2]);
    EXPECT_FLOAT_EQ(6, out[1][0]);
    EXPECT_FLOAT_EQ(-12, out[1][1]);
    EXPECT_FLOAT_EQ(6, out[1][2]);
    EXPECT_FLOAT_EQ(-3, out[2][0]);
    EXPECT_FLOAT_EQ(6, out[2][1]);
    EXPECT_FLOAT_EQ(-3, out[2][2]);
}

// https://www.wolframalpha.com/input/?i=adjoint+%7B%7B1%2C2%2C3%2C4%7D%2C%7B5%2C6%2C7%2C8%7D%2C%7B9%2C10%2C11%2C12%7D%2C%7B13%2C14%2C15%2C16%7D%7D&assumption=%7B%22C%22%2C+%22adjoint%22%7D+-%3E+%7B%22LinearAlgebraWord%22%7D
TEST(MathUserUtils, adjointFourByFourZero)
{
    const float aArr[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    modm::Matrix4f a(aArr);
    modm::Matrix4f out;

    EXPECT_TRUE(adjoint(a, &out));
    EXPECT_FLOAT_EQ(0, out[0][0]);
    EXPECT_FLOAT_EQ(0, out[0][1]);
    EXPECT_FLOAT_EQ(0, out[0][2]);
    EXPECT_FLOAT_EQ(0, out[0][3]);
    EXPECT_FLOAT_EQ(0, out[1][0]);
    EXPECT_FLOAT_EQ(0, out[1][1]);
    EXPECT_FLOAT_EQ(0, out[1][2]);
    EXPECT_FLOAT_EQ(0, out[1][3]);
    EXPECT_FLOAT_EQ(0, out[2][0]);
    EXPECT_FLOAT_EQ(0, out[2][1]);
    EXPECT_FLOAT_EQ(0, out[2][2]);
    EXPECT_FLOAT_EQ(0, out[2][3]);
    EXPECT_FLOAT_EQ(0, out[3][0]);
    EXPECT_FLOAT_EQ(0, out[3][1]);
    EXPECT_FLOAT_EQ(0, out[3][2]);
    EXPECT_FLOAT_EQ(0, out[3][3]);
}

// https://www.wolframalpha.com/input/?i=adjoint+%7B%7B-12%2C5%2C-8%2C4%7D%2C%7B5%2C6%2C7%2C8%7D%2C%7B9%2C10%2C11%2C12%7D%2C%7B13%2C14%2C15%2C16%7D%7D&assumption=%7B%22C%22%2C+%22adjoint%22%7D+-%3E+%7B%22LinearAlgebraWord%22%7D
TEST(MathUserUtils, adjointFourByFourNonzero)
{
    const float aArr[] = {-12, 5, -8, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    modm::Matrix4f a(aArr);
    modm::Matrix4f out;

    EXPECT_TRUE(adjoint(a, &out));
    EXPECT_FLOAT_EQ(0, out[0][0]);
    EXPECT_FLOAT_EQ(100, out[0][1]);
    EXPECT_FLOAT_EQ(-200, out[0][2]);
    EXPECT_FLOAT_EQ(100, out[0][3]);
    EXPECT_FLOAT_EQ(0, out[1][0]);
    EXPECT_FLOAT_EQ(-80, out[1][1]);
    EXPECT_FLOAT_EQ(160, out[1][2]);
    EXPECT_FLOAT_EQ(-80, out[1][3]);
    EXPECT_FLOAT_EQ(0, out[2][0]);
    EXPECT_FLOAT_EQ(-140, out[2][1]);
    EXPECT_FLOAT_EQ(280, out[2][2]);
    EXPECT_FLOAT_EQ(-140, out[2][3]);
    EXPECT_FLOAT_EQ(0, out[3][0]);
    EXPECT_FLOAT_EQ(120, out[3][1]);
    EXPECT_FLOAT_EQ(-240, out[3][2]);
    EXPECT_FLOAT_EQ(120, out[3][3]);
}

TEST(MathUserUtils, inverseOneByOne)
{
    const float a[]{1};
    modm::Matrix1f aMat(a);
    modm::Matrix1f out;
    EXPECT_TRUE(inverse(aMat, &out));
    EXPECT_EQ(1, out[0][0]);
}

// https://www.wolframalpha.com/input/?i=inverse+matrix+%7B%7B1%2C2%7D%2C%7B3%2C4%7D%7D
TEST(MathUserUtils, inverseTwoByTwo)
{
    const float a[]{1, 2, 3, 4};
    modm::Matrix2f aMat(a);
    modm::Matrix2f out;

    EXPECT_TRUE(inverse(aMat, &out));
    EXPECT_FLOAT_EQ(-2, out[0][0]);
    EXPECT_FLOAT_EQ(1, out[0][1]);
    EXPECT_FLOAT_EQ(1.5f, out[1][0]);
    EXPECT_FLOAT_EQ(-0.5f, out[1][1]);
}

// https://www.wolframalpha.com/input/?i=inverse+matrix+%7B%7B-3.12%2C+50.4%7D%2C%7B-9.9%2C+23.23%7D%7D
TEST(MathUserUtils, inverseTwoByTwoDecimalsAndNegatives)
{
    const float a[]{-3.12f, 50.4f, -9.9f, 23.23f};
    modm::Matrix2f aMat(a);
    modm::Matrix2f out;

    EXPECT_TRUE(inverse(aMat, &out));
    EXPECT_TRUE(compareFloatClose(0.0544688, out[0][0], 0.0001f));
    EXPECT_TRUE(compareFloatClose(-0.118176, out[0][1], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.0232132, out[1][0], 0.0001f));
    EXPECT_TRUE(compareFloatClose(-0.00731566, out[1][1], 0.0001f));
}

// https://www.wolframalpha.com/input/?i=inverse+matrix+%7B%7B-3.12%2C+50.4%2C-9.9%2C+23.23%7D%2C%7B32.54%2C+-65%2C+12.9%2C-65%7D%2C%7B-90%2C+120.9%2C+34.5%2C+235%7D%2C%7B12%2C+13%2C+-345%2C+34.7%7D%7D
TEST(MathUserUtils, inverseFourByFourDecimalsAndNegatives)
{
    const float a[]{
        -3.12,
        50.4,
        -9.9,
        23.23,
        32.54,
        -65,
        12.9,
        -65,
        -90,
        120.9,
        34.5,
        235,
        12,
        13,
        -345,
        34.7};
    modm::Matrix4f aMat(a);
    modm::Matrix4f out;

    EXPECT_TRUE(inverse(aMat, &out));
    EXPECT_TRUE(compareFloatClose(0.0623085, out[0][0], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.07553, out[0][1], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.0143669, out[0][2], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.00247287, out[0][3], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.0181185, out[1][0], 0.0001f));
    EXPECT_TRUE(compareFloatClose(-0.00941376, out[1][1], 0.0001f));
    EXPECT_TRUE(compareFloatClose(-0.00420401, out[1][2], 0.0001f));
    EXPECT_TRUE(compareFloatClose(-0.00129232, out[1][3], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.0042498, out[2][0], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.00558645, out[2][1], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.00151784, out[2][2], 0.0001f));
    EXPECT_TRUE(compareFloatClose(-0.00265983, out[2][3], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.0139176, out[3][0], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.0329493, out[3][1], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.0116975, out[3][2], 0.0001f));
    EXPECT_TRUE(compareFloatClose(0.0020024, out[3][3], 0.0001f));
}
