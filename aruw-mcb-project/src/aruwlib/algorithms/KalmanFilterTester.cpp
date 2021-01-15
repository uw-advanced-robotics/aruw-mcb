#ifdef PLATFORM_HOSTED

#include "KalmanFilterTester.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

const char KalmanFilterTester::TEST_FILE_LOCATION[] = "./kalman-test-files/";

KalmanFilterTester::KalmanFilterTester() {}

void KalmanFilterTester::runTests()
{
    std::ifstream file(std::string(TEST_FILE_LOCATION) + "test.csv");

    if (file)
    {
        std::string line;

        std::vector<float> vxResults;
        std::vector<float> vyResults;
        std::vector<float> yawIMUResults;
        std::vector<float> omegaWheelResults;
        std::vector<float> omegaIMUResults;

        while (std::getline(file, line))
        {
            float vx = 0;
            float vy = 0;
            float yawIMU = 0;
            float omegaWheel = 0;
            float omegaIMU = 0;

            std::istringstream in(line);

            char delimeter;
            in >> vx >> delimeter >> vy >> delimeter >> yawIMU >> delimeter >> omegaWheel >>
                delimeter >> omegaIMU;

            filterToTest.update(vx, vy, yawIMU, omegaWheel, omegaIMU);

            const auto &x = filterToTest.getX();

            vxResults.push_back(x[0][0]);
            vyResults.push_back(x[1][0]);
            yawIMUResults.push_back(x[2][0]);
            omegaWheelResults.push_back(x[3][0]);
            omegaIMUResults.push_back(x[4][0]);
        }

        file.close();

        plt::plot(vxResults);
        plt::show();
    }
    else
    {
        std::cout << "File not found" << std::endl;
    }
}

#endif  // PLATFORM_HOSTED
