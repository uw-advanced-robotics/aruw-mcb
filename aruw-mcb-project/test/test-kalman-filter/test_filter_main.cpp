#include <iostream>

#include <gtest/gtest.h>

#include "two_dim_filter.hpp"

TEST(t, tt)
{
    TwoDimFilter f;
    float vxWheels = 10;
    float vyWheels = 10;
    float yawIMU = 0;
    float omegaWheels = 0;
    float omegaIMU = 0;
    for (int i = 0; i < 100; i++)
    {
        f.update(vxWheels, vyWheels, yawIMU, omegaWheels, omegaIMU);
        const modm::Matrix<float, 6, 1> &x = f.getX();
        std::cout << x[0][0] << ", " << x[1][0] << ", " << x[2][0] << ", " << x[3][0] << ", "
                  << x[4][0] << ", " << x[5][0] << std::endl;
    }
}