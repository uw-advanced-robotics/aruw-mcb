#include "turret_pid.hpp"

#include <aruwlib/algorithms/math_user_utils.hpp>
#include <aruwlib/architecture/clock.hpp>

using namespace aruwlib::algorithms;

namespace aruwsrc
{
namespace algorithms
{
float TurretPid::runController(float error, float errorDerivative, float dt)
{
    // p
    currErrorP = kp * proportionalKalman.filterData(error);
    // i
    currErrorI = limitVal<float>(
        currErrorI + ki * proportionalKalman.getLastFiltered() * dt,
        -maxICumulative,
        maxICumulative);
    // d
    currErrorD = -kd * derivativeKalman.filterData(errorDerivative);
    // total
    output = limitVal<float>(currErrorP + currErrorI + currErrorD, -maxOutput, maxOutput);
    return output;
}

float TurretPid::runControllerDerivateError(float error, float dt)
{
    float errorDerivative = (error - prevError) / dt;
    prevError = error;
    return runController(error, errorDerivative, dt);
}

float TurretPid::getOutput() { return output; }

void TurretPid::reset()
{
    this->output = 0.0f;
    this->currErrorP = 0.0f;
    this->currErrorI = 0.0f;
    this->currErrorD = 0.0f;
    this->derivativeKalman.reset();
    this->proportionalKalman.reset();
}

}  // namespace algorithms

}  // namespace aruwsrc
