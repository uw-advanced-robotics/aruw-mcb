/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "smooth_pid.hpp"

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;

namespace tap
{
namespace algorithms
{
float SmoothPid::runController(float error, float errorDerivative, float dt)
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

float SmoothPid::runControllerDerivateError(float error, float dt)
{
    float errorDerivative = (error - prevError) / dt;
    prevError = error;
    return runController(error, errorDerivative, dt);
}

float SmoothPid::getOutput() { return output; }

void SmoothPid::reset()
{
    this->output = 0.0f;
    this->currErrorP = 0.0f;
    this->currErrorI = 0.0f;
    this->currErrorD = 0.0f;
    this->derivativeKalman.reset();
    this->proportionalKalman.reset();
}

}  // namespace algorithms

}  // namespace tap
