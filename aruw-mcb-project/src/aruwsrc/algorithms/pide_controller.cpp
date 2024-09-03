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

#include "pide_controller.hpp"

namespace aruwsrc::algorithms
{
PIDEController::PIDEController(Config& config) : config(config) {}

float PIDEController::runController(float measurement, float dt)
{
    float error = setpoint - measurement;
    current_value_integral += error * dt;
    current_value_integral =
        std::clamp(current_value_integral, -config.max_integral, config.max_integral);

    float pidCompensation = config.kp * error + config.ki * current_value_integral +
                            config.kd * (error - last_error) / dt;

    if (config.feedforward != 0)
    {
        output = config.feedforward * setpoint + pidCompensation;
    }
    else
    {
        output += pidCompensation;
    }

    last_error = error;
    return std::clamp(output, -config.max_output, config.max_output);
}

}  // namespace aruwsrc::algorithms
