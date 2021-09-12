#ifndef MATRIX_TEST_HPP_
#define MATRIX_TEST_HPP_

#ifndef PLATFORM_HOSTED

#include "modm/math/matrix.hpp"

#include "arm_math.h"
#include "kalman_filter.hpp"
#include "matrix_utils.hpp"

namespace aruwlib
{
class Drivers;
}

namespace aruwsrc::algorithms
{
/**
 * Some code that profiles the DSP module
 */
class MatrixTest
{
public:
    MatrixTest();
    ~MatrixTest() = default;

    void smallMultiplication();

    void multiplicationSpeedCompare();
    void additionSpeedCompare();
    void inverseSpeedCompare();
    void transposeSpeedCompare();

    void smallKalmanFilter();

private:
    static constexpr int ROWS = 10;
    static constexpr int COLS = 10;
    static constexpr int TRIALS = 100;
    uint32_t timeCmsis[TRIALS], timeModm[TRIALS];
    CMSISMat<ROWS, COLS> cmsisMat1;
    CMSISMat<ROWS, COLS> cmsisMat2;
    CMSISMat<ROWS, COLS> cmsisMat3;
    modm::Matrix<float, ROWS, COLS> modmMat1;
    modm::Matrix<float, ROWS, COLS> modmMat2;
    modm::Matrix<float, ROWS, COLS> modmMat3;
};

}  // namespace aruwsrc::algorithms

#endif

#endif  // MATRIX_TEST_HPP_
