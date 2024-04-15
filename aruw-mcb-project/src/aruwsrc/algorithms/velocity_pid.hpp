/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VELOCITY_PID_HPP_
#define VELOCITY_PID_HPP_

#include <float.h>
#include <stdint.h>

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc::algorithms
{
/**
 * A PID controller for velocity control. Running the control does not set output based on error,
 * but instead changes the output.
 */
class VelocityPid
{
public:
    struct Config
    {
        float kp;
        float ki;
        float kd;
        float feedforward;
        float max_output;
        float max_integral;
        float error_deadband;
    };

    VelocityPid(Config& config);

    float runController(float error);

    float runController(float error, float error_derivative, float feedforward, float dt);

private:
    Config& config_;
    float output = 0;
    float current_value_integral = 0;
};

}  // namespace aruwsrc::algorithms

#endif
