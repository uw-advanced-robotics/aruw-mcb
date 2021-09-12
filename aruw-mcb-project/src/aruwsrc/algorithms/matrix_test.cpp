#ifndef PLATFORM_HOSTED

#include "matrix_test.hpp"

#include "tap/architecture/clock.hpp"

using namespace tap::arch::clock;

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
    const float A[STATES * STATES] = {1};
    const float C[INPUTS * STATES] = {.5, .5};
    const float Q[STATES * STATES] = {1};
    const float R[INPUTS * INPUTS] = {1, 0, 0, 1};
    const float P[STATES * STATES] = {1};
    KalmanFilter<STATES, INPUTS> kf(A, C, Q, R, P);
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

        uint32_t startTime = getTimeMicroseconds();
        arm_mat_mult_f32(&cmsisMat1.matrix, &cmsisMat2.matrix, &cmsisMat3.matrix);
        timeCmsis[k] = getTimeMicroseconds() - startTime;

        startTime = getTimeMicroseconds();
        modmMat3 = modmMat1 * modmMat2;
        timeModm[k] = getTimeMicroseconds() - startTime;
    }
}

void MatrixTest::additionSpeedCompare()
{
    for (int k = 0; k < TRIALS; k++)
    {
        populateMatrices<ROWS, COLS>(cmsisMat1, modmMat1);
        populateMatrices<ROWS, COLS>(cmsisMat2, modmMat2);

        uint32_t startTime = getTimeMicroseconds();
        arm_mat_add_f32(&cmsisMat1.matrix, &cmsisMat2.matrix, &cmsisMat3.matrix);
        timeCmsis[k] = getTimeMicroseconds() - startTime;

        startTime = getTimeMicroseconds();
        modmMat3 = modmMat1 + modmMat2;
        timeModm[k] = getTimeMicroseconds() - startTime;
    }
}

void MatrixTest::inverseSpeedCompare()
{
    for (int k = 0; k < TRIALS; k++)
    {
        populateMatrices<ROWS, COLS>(cmsisMat1, modmMat1);

        uint32_t startTime = getTimeMicroseconds();
        arm_mat_inverse_f32(&cmsisMat1.matrix, &cmsisMat2.matrix);
        timeCmsis[k] = getTimeMicroseconds() - startTime;
    }
}

void MatrixTest::transposeSpeedCompare()
{
    for (int k = 0; k < TRIALS; k++)
    {
        populateMatrices<ROWS, COLS>(cmsisMat1, modmMat1);

        uint32_t startTime = getTimeMicroseconds();
        arm_mat_trans_f32(&cmsisMat1.matrix, &cmsisMat2.matrix);
        timeCmsis[k] = getTimeMicroseconds() - startTime;

        startTime = getTimeMicroseconds();
        modmMat2 = modmMat1.asTransposed();
        timeModm[k] = getTimeMicroseconds() - startTime;
    }
}

}  // namespace aruwsrc::algorithms

#endif
