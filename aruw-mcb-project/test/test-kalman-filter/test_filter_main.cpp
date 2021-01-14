#include <iostream>

#include <gtest/gtest.h>

#include "two_dim_filter.hpp"

TEST(t, tt)
{
    TwoDimFilter f;
    float vxWheels = 1000;
    float vyWheels = 1000;
    float yawIMU = 0;
    float omegaWheels = 0;
    float omegaIMU = 0;
    for (int i = 0; i < 500; i++)
    {
        if (yawIMU < aruwlib::algorithms::PI / 2) {
            omegaWheels = aruwlib::algorithms::PI / 0.064;
            omegaIMU = aruwlib::algorithms::PI / 0.064;
            yawIMU += aruwlib::algorithms::PI / 32;
        } else {
            omegaWheels = 0;
            omegaIMU = 0;
        }
        f.update(vxWheels, vyWheels, yawIMU, omegaWheels, omegaIMU);
        // const modm::Matrix<float, 6, 1> &x = f.getX();
    }
}