/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
 * A PIDF controller for velocity control.
 * 
 * This controller works in 2 ways, if there is a feedforward term, it acts as a position PID controller, 
 */
class PIDEController
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
    };

    PIDEController(Config& config);

    float setSetpoint(float setpoint)
    {
        if(this->setpoint != setpoint){
            current_value_integral = 0;
        }
        this->setpoint = setpoint;
    }

    float runController(float measurement, float dt);

private:
    Config& config;
    float setpoint = 0;
    float output = 0;
    float current_value_integral = 0;
    float last_error = 0;
};

}  // namespace aruwsrc::algorithms

#endif
