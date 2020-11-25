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
#ifndef motor_sim_m3508_hpp_

#define motor_sim_m3508_hpp_

#include <time.h>

#include <cstdint>

namespace aruwlib
{
namespace motor
{
class MotorSimM3508
{
public:
    MotorSimM3508();

    /**
     * Returns the current given to the GM3508 for a given integer input.
     * Domain: -16384 ~ 0 ~ 16384
     * Output Range: -20 A ~ 0 ~ 20 A
     * It should be noted that the rated current for the GM3508 is 10 A.
     */
    static float getCurrent(int16_t in);

    /**
     * Returns the maximum torque (in N*m) that can be held in equilibrium for a given integer input.
     */
    static float getMaxTorque(int16_t in);

    /**
     * Returns the current number of rotations the motor has undergone
     * since the beginning of the simulation.
     */
    float getEnc();

    /**
     * Returns the current rotational speed of the motor in RPM.
     */
    float getRPM();

    /**
     * Updates the relevant quantities for the motor being simulated.
     * Must be run iteratively in order for getEnc() and getRPM() to work correctly.
     */
    void update(int16_t in, float loading);

private:
    /* Constants */
    static constexpr float MAX_INPUT_MAG = 16384;  // Integer
    static constexpr float MAX_CURRENT_OUT = 20;   // Amps
    static constexpr float CURRENT_LIM = 10;       // Amps
    static constexpr float MAX_W = 469;            // RPM
    static constexpr float KT = 0.3;               // (N*m)/A
    static constexpr float WT_GRAD = 72;           // RPM/(N*m)

    /* Class Variables */
    float pos;                                     // Meters
    float rpm;                                     // RPM
    clock_t time;
};
}  // namespace motor
}  // namespace aruwlib
#endif
#endif