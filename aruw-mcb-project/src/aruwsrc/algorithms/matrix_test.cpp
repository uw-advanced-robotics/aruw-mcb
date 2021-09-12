#include "matrix_test.hpp"

#include "aruwlib/architecture/clock.hpp"

#ifndef PLATFORM_HOSTED

namespace aruwsrc::algorithms
{
#define EXPECT_TRUE(expr) modm_assert((expr), #expr, __line__)

MatrixTest::MatrixTest() {}

void MatrixTest::smallMultiplication()
{
    CMSISMat<2, 2> smallMatrixA;
    CMSISMat<2, 2> smallMatrixB;
    CMSISMat<2, 2> smallMatrixC;

    smallMatrixA.data[0] = 1;
    smallMatrixA.data[1] = 2;
    smallMatrixA.data[2] = 3;
    smallMatrixA.data[3] = 4;

    smallMatrixB.data[0] = 5;
    smallMatrixB.data[1] = 6;
    smallMatrixB.data[2] = 7;
    smallMatrixB.data[3] = 8;

    arm_mat_mult_f32(&smallMatrixA.matrix, &smallMatrixB.matrix, &smallMatrixC.matrix);

    EXPECT_TRUE(smallMatrixC.data[0] == 19);
    EXPECT_TRUE(smallMatrixC.data[1] == 22);
    EXPECT_TRUE(smallMatrixC.data[2] == 43);
    EXPECT_TRUE(smallMatrixC.data[3] == 50);
}

void MatrixTest::smallKalmanFilter()
{
    const float x[2] = {0, 0};
    const float P[4] = {0, 0, 0, 0};
    const float A[4] = {0, 0, 0, 0};
    const float B[4] = {0, 0, 0, 0};
    const float C[4] = {0, 0, 0, 0};
    const float Q[4] = {0, 0, 0, 0};
    const float R[4] = {0, 0, 0, 0};
    KalmanFilter<2, 2> smallKalmanFilter(x, P, A, B, C, Q, R);
    CMSISMat<2, 1> z;
    z.data[0] = 1;
    z.data[1] = 1;
    smallKalmanFilter.performUpdate(z);
}

template <uint16_t ROWS, uint16_t COLS>
static void populateMatrices(
    CMSISMat<ROWS, COLS> &cmsisMat,
    modm::Matrix<float, ROWS, COLS> &modmMat)
{
    // Fill matrices
    for (int i = 0; i < ROWS; i++)
    {
        for (int j = 0; j < COLS; j++)
        {
            float randNum = static_cast<float>(rand()) / 100.0f;
            cmsisMat.data[i * ROWS + j] = randNum;
            modmMat[i][j] = randNum;
        }
    }
}

void MatrixTest::multiplicationSpeedCompare()
{
    for (int k = 0; k < TRIALS; k++)
    {
        populateMatrices<ROWS, COLS>(cmsisMat1, modmMat1);
        populateMatrices<ROWS, COLS>(cmsisMat2, modmMat2);

        uint32_t startTime = aruwlib::arch::clock::getTimeMicroseconds();
        arm_mat_mult_f32(&cmsisMat1.matrix, &cmsisMat2.matrix, &cmsisMat3.matrix);
        timeCmsis[k] = aruwlib::arch::clock::getTimeMicroseconds() - startTime;

        startTime = aruwlib::arch::clock::getTimeMicroseconds();
        modmMat3 = modmMat1 * modmMat2;
        timeModm[k] = aruwlib::arch::clock::getTimeMicroseconds() - startTime;
    }
}

void MatrixTest::additionSpeedCompare()
{
    for (int k = 0; k < TRIALS; k++)
    {
        populateMatrices<ROWS, COLS>(cmsisMat1, modmMat1);
        populateMatrices<ROWS, COLS>(cmsisMat2, modmMat2);

        uint32_t startTime = aruwlib::arch::clock::getTimeMicroseconds();
        arm_mat_add_f32(&cmsisMat1.matrix, &cmsisMat2.matrix, &cmsisMat3.matrix);
        timeCmsis[k] = aruwlib::arch::clock::getTimeMicroseconds() - startTime;

        startTime = aruwlib::arch::clock::getTimeMicroseconds();
        modmMat3 = modmMat1 + modmMat2;
        timeModm[k] = aruwlib::arch::clock::getTimeMicroseconds() - startTime;
    }
}

void MatrixTest::inverseSpeedCompare()
{
    for (int k = 0; k < TRIALS; k++)
    {
        populateMatrices<ROWS, COLS>(cmsisMat1, modmMat1);

        uint32_t startTime = aruwlib::arch::clock::getTimeMicroseconds();
        arm_mat_inverse_f32(&cmsisMat1.matrix, &cmsisMat2.matrix);
        timeCmsis[k] = aruwlib::arch::clock::getTimeMicroseconds() - startTime;
    }
}

void MatrixTest::transposeSpeedCompare()
{
    for (int k = 0; k < TRIALS; k++)
    {
        populateMatrices<ROWS, COLS>(cmsisMat1, modmMat1);

        uint32_t startTime = aruwlib::arch::clock::getTimeMicroseconds();
        arm_mat_trans_f32(&cmsisMat1.matrix, &cmsisMat2.matrix);
        timeCmsis[k] = aruwlib::arch::clock::getTimeMicroseconds() - startTime;

        startTime = aruwlib::arch::clock::getTimeMicroseconds();
        modmMat2 = modmMat1.asTransposed();
        timeModm[k] = aruwlib::arch::clock::getTimeMicroseconds() - startTime;
    }
}

}  // namespace aruwsrc::algorithms

#endif
