/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifdef PLATFORM_HOSTED

#include "motor_sim_m3508.hpp"

#include <time.h>

#include <cstdint>

namespace aruwlib
{
namespace motor
{
MotorSimM3508::MotorSimM3508() : pos(0) { time = clock(); }

float MotorSimM3508::getCurrent(int16_t in) { return (in / MAX_INPUT_MAG) * MAX_CURRENT_OUT; }

float MotorSimM3508::getMaxTorque(int16_t in) { return getCurrent(in) * KT; }

float MotorSimM3508::getEnc() { return pos; }

float MotorSimM3508::getRPM() { return rpm; }

// TODO: Make sure that position simulation is accurate (potential timer issues, encoder tics?)
void MotorSimM3508::update(int16_t in, float loading)
{
    rpm = (MAX_W - WT_GRAD * loading) * in / (MAX_INPUT_MAG * (CURRENT_LIM / MAX_CURRENT_OUT));
    pos = (static_cast<float>(clock() - time) / CLOCKS_PER_SEC) * rpm;
    time = clock();
}
}  // namespace motor
}  // namespace aruwlib
#endif