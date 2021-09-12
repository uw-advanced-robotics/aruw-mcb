#include <iostream>

#include <gtest/gtest.h>

#include "aruwsrc/algorithms/matrix_utils.hpp"

#include "arm_math.h"

using namespace aruwsrc::algorithms;

TEST(CMSISMat, default_constructor_matrix_zeroed_cmsis_mat_inited)
{
    CMSISMat<2, 1> m;

    EXPECT_FLOAT_EQ(0, m.data[0]);
    EXPECT_FLOAT_EQ(0, m.data[1]);
    EXPECT_EQ(m.data, m.matrix.pData);
    EXPECT_EQ(2, m.matrix.numRows);
    EXPECT_EQ(1, m.matrix.numCols);
}

TEST(CMSISMat, onearg_constructor_matrix_equal_to_passed_in_matrix)
{
    float arr[2 * 1];
    arr[0] = 10;
    arr[1] = 20;
    CMSISMat<2, 1> m(arr);

    EXPECT_FLOAT_EQ(10, m.data[0]);
    EXPECT_FLOAT_EQ(20, m.data[1]);
    EXPECT_EQ(m.data, m.matrix.pData);
    EXPECT_EQ(2, m.matrix.numRows);
    EXPECT_EQ(1, m.matrix.numCols);

    // Changing arr shouldn't affect stuff stored in m since we copied the
    // data
    arr[0] = 1;
    EXPECT_FLOAT_EQ(10, m.data[0]);
}

TEST(CMSISMat, addop_simple)
{
    CMSISMat<2, 1> a;
    CMSISMat<2, 1> b;

    a.data[0] = 1;
    a.data[1] = 2;

    b.data[0] = 3;
    b.data[1] = 4;

    CMSISMat c = a + b;

    EXPECT_FLOAT_EQ(4, c.data[0]);
    EXPECT_FLOAT_EQ(6, c.data[1]);
}

TEST(CMSISMat, subop_simple)
{
    CMSISMat<2, 1> a;
    CMSISMat<2, 1> b;

    a.data[0] = 1;
    a.data[1] = 2;

    b.data[0] = 3;
    b.data[1] = 4;

    CMSISMat c = a - b;

    EXPECT_FLOAT_EQ(-2, c.data[0]);
    EXPECT_FLOAT_EQ(-2, c.data[1]);
}

TEST(CMSISMat, multop_simple)
{
    CMSISMat<2, 1> a;
    CMSISMat<1, 2> b;

    a.data[0] = 1;
    a.data[1] = 2;

    b.data[0] = 3;
    b.data[1] = 4;

    CMSISMat c = a * b;

    EXPECT_FLOAT_EQ(3, c.data[0]);
    EXPECT_FLOAT_EQ(4, c.data[1]);
    EXPECT_FLOAT_EQ(6, c.data[2]);
    EXPECT_FLOAT_EQ(8, c.data[3]);
}
