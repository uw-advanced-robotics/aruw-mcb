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
    currErrorD = kd * derivativeKalman.filterData(rotationalSpeed);
    // total
    output = limitVal<float>(currErrorP + currErrorI - currErrorD, -maxOutput, maxOutput);
    return output;
}

float TurretPid::getValue()
{
    return output;
}

}

}
