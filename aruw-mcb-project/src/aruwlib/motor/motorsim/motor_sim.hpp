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
#ifndef motor_sim_hpp_

#define motor_sim_hpp_

#include <time.h>

#include <cstdint>

namespace aruwlib
{
namespace motorsim
{
class MotorSim
{
public:
    /**
     * Enum type representing the different types of motors that can be simulated.
     */
    enum MotorType
    {
        M3508,
        GM6020
    };

    MotorSim(MotorType type);
    MotorSim(MotorType type, float loading);

    void reset();

    /**
     * Returns the current (in amps) given to the GM3508 for the given input.
     */
    float getActualCurrent();

    /**
     * Returns the angular position of the motor.
     */
    int16_t getEnc();

    /**
     * Returns the input integer given to the motor by the CAN messages.
     */
    int16_t getInput();

    /**
     * Returns the maximum torque (in N*m) that can be held in equilibrium for the given input.
     */
    float getMaxTorque();

    /**
     * Returns the current rotational speed of the motor in RPM.
     */
    int16_t getRPM();

    /**
     * Sets the input (as an integer) used for simulation.
     * Should be updated every cycle. Default input is 0.
     * Function will do nothing if input is invalid.
     */
    void setInput(int16_t in);

    /**
     * Sets the loading on the motor (in N*m) used for simulation.
     * The default loading value is 0 N*m.
     * Function will do nothing if input is greater than rated torque.
     */
    void setLoading(float t);

    /**
     * Updates the relevant quantities for the motor being simulated.
     * Must be run iteratively in order for getEnc() and getRPM() to work correctly.
     */
    void update();

private:
    /* Constants */
    /* Note that these should all be constexpr, but because of
    how they are initialized in construction they cannot be. */
    int16_t MAX_INPUT_MAG = 0;    // Integer
    int16_t MAX_ENCODER_VAL = 0;  // Integer
    float MAX_CURRENT_OUT = 0;    // Amps
    float CURRENT_LIM = 0;        // Amps
    float MAX_W = 0;              // RPM
    float KT = 0;                 // (N*m)/A
    float WT_GRAD = 0;            // RPM/(N*m)

    /** Initializes constant variables based on motor type */
    void initConstants(MotorType type);

    /* Class Variables */
    float loading;  // N*m
    float pos;      // Meters
    int16_t rpm;    // RPM
    int16_t input;  // 16-bit Integer
    clock_t time;
};
}  // namespace motorsim
}  // namespace aruwlib
#endif
#endif