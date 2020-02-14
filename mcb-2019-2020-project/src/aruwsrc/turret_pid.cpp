#include <rm-dev-board-a/board.hpp>
#include "turret_pid.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

using namespace aruwlib::algorithms;

namespace aruwsrc
{

namespace algorithms
{

float TurretPid::runController(float angleError, float rotationalSpeed)
{
    // p
    currErrorP = kp * proportionalKalman.filterData(angleError);
    // i
    currErrorI = limitVal<float>(currErrorI + ki * proportionalKalman.getLastFiltered(),
        -maxICumulative, maxICumulative);
    // d
    // currErrorD = -kd * limitVal<float>(
    //     derivativeKalman.filterData(rotationalSpeed), -maxD, maxD
    // );
    // currAverageBeforeDividing -= derivativePreviousStore[averageDIndex];  // todo fix
    derivativePreviousStore[averageDIndex] = -kd *  // todo try out this version
        limitVal<float>(derivativeKalman.filterData(
            angleError /
            (static_cast<float>(Board::getTimeMicroseconds() - prevTimeMicroseconds)) / 1000.0f),
            -maxD, maxD
        );

    derivativePreviousStore[averageDIndex] = -kd * limitVal<float>(
        derivativeKalman.filterData(rotationalSpeed), -maxD, maxD);

    currAverageBeforeDividing = 0.0f;

    for (int i = 0; i < 10; i++) {  // todo fix
        currAverageBeforeDividing += derivativePreviousStore[i];
    }
    
    // currAverageBeforeDividing += derivativePreviousStore[averageDIndex];
    currErrorD = currAverageBeforeDividing / 10.0f;
    averageDIndex = (averageDIndex + 1) % derivativeStepsToAverage;
    // total
    output = limitVal<float>(currErrorP + currErrorI + currErrorD, -maxOutput, maxOutput);
    lowPassOutput = lowPassAlpha * output + (1 - lowPassAlpha) * lowPassOutput;
    return lowPassOutput;
}

}

}
