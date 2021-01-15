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
    std::ifstream file(std::string(TEST_FILE_LOCATION) + "imu_data.csv");

    if (file)
    {
        std::string line;

        std::vector<float> timeData;
        std::vector<float> vxData;
        std::vector<float> vyData;
        std::vector<float> yawIMUData;
        std::vector<float> omegaWheelData;
        std::vector<float> omegaIMUData;

        std::vector<float> xPosResults;
        std::vector<float> yPosResults;
        std::vector<float> vxResults;
        std::vector<float> vyResults;
        std::vector<float> yawResults;
        std::vector<float> omegaResults;

        float prevTime = -100000;
        std::getline(file, line);  // throw away first line
        while (std::getline(file, line))
        {
            int index = 0;
            float time = 0;
            float vx = 0;
            float vy = 0;
            float yawIMU = 0;
            float omegaWheel = 0;
            float omegaIMU = 0;

            std::istringstream in(line);

            char delimeter;
            // in >> index >> delimeter >> time >> delimeter >> vx >> delimeter >> vy >> delimeter >>
            //     yawIMU >> delimeter >> omegaWheel >> delimeter >> omegaIMU;
            in >> index >> delimeter >> time >> delimeter >> omegaIMU >> delimeter >> yawIMU;

            std::cout << time << ", " << yawIMU << ", " << omegaWheel << std::endl;

            if (time - prevTime > 0.002f)
            {
                timeData.push_back(time);
                vxData.push_back(vx);
                vyData.push_back(vy);
                yawIMUData.push_back(yawIMU);
                omegaWheelData.push_back(omegaWheel);
                omegaIMUData.push_back(omegaIMU);
                
                filterToTest.update(vx, vy, yawIMU, omegaWheel, omegaIMU);
                const auto &x = filterToTest.getX();

                xPosResults.push_back(x[0][0]);
                yPosResults.push_back(x[1][0]);
                vxResults.push_back(x[2][0]);
                vyResults.push_back(x[3][0]);
                yawResults.push_back(x[4][0]);
                omegaResults.push_back(x[5][0]);
                prevTime = time;
            }
        }

        file.close();

        plt::named_plot("omega IMU", timeData, omegaIMUData);
        plt::named_plot("omega filtered", timeData, omegaResults);
        plt::legend();
        plt::title("rotational velocity, raw imu vs. kalman filtered");
        plt::show();
    }
    else
    {
        std::cout << "File not found" << std::endl;
    }
}

#endif  // PLATFORM_HOSTED
