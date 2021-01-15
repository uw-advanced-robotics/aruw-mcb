#ifdef PLATFORM_HOSTED

#ifndef KALMAN_FILTER_TESTER_HPP_
#define KALMAN_FILTER_TESTER_HPP_

#include "two_dim_filter.hpp"

class KalmanFilterTester
{
public:
    KalmanFilterTester();

    void runTests();

private:
    static const char TEST_FILE_LOCATION[];

    TwoDimFilter filterToTest;
};

#endif  // KALMAN_FILTER_TESTER_HPP_

#endif  // PLATFORM_HOSTED
