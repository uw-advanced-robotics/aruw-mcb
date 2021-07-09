#ifndef MATRIX_TEST_HPP_
#define MATRIX_TEST_HPP_

#ifndef PLATFORM_HOSTED

#include "arm_math.h"
#include "kalman_filter.hpp"
#include "matrix_utils.hpp"

namespace aruwlib
{
class Drivers;
}

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
};

#endif

#endif  // MATRIX_TEST_HPP_
