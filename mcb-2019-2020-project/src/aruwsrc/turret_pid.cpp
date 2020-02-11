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
    currAverageBeforeDividing -= derivativePreviousStore[averageDIndex];
    derivativePreviousStore[averageDIndex] = -kd * limitVal<float>(
        derivativeKalman.filterData(rotationalSpeed), -maxD, maxD);
    currAverageBeforeDividing += derivativePreviousStore[averageDIndex];
    currErrorD = currAverageBeforeDividing / 10.0f;
    averageDIndex = (averageDIndex + 1) % derivativeStepsToAverage;
    // total
    output = limitVal<float>(currErrorP + currErrorI + currErrorD, -maxOutput, maxOutput);
    lowPassOutput = lowPassAlpha * output + (1 - lowPassAlpha) * lowPassOutput;
    return lowPassOutput;
}

}

}
