/*
 * Copyright (c) 2020-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef TURRET_STOS_CONTROLLER_HPP_
#define TURRET_STOS_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

#include "modm/math/geometry.hpp"

// See this paper: https://ieeexplore.ieee.org/document/1264127
// This is the implementation for case 1.
namespace aruwsrc::control::turret::algorithms
{
class OptimalSTOSController
{
private:
    float a, U_b2;
    float a1, a2, T_d;
    float E_pos, E_neg;
    float S1_const, S4_const, x_star_const;

    float J_TOTAL;
    float TAU_MAX;
    float B_DAMP;
    float W_D;
    float ZETA;
    float SYSTEM_DELAY_SEC;
    float TorqueToMotorOutput;

public:
    struct STOSConstants
    {
        float J_TOTAL;           // Rigid body inertia
        float TAU_MAX;           // Max torque
        float B_DAMP;            // Rigid body damping
        float W_D;               // Damped natural frequency of flexible system
        float ZETA;              // Damping ratio of the flexible system
        float SYSTEM_DELAY_SEC;  // Total system delay (sensing + computation + actuation)
        float TorqueToMotorOutput;
    };

    OptimalSTOSController(STOSConstants constants)
        : J_TOTAL(constants.J_TOTAL),
          TAU_MAX(constants.TAU_MAX),
          B_DAMP(constants.B_DAMP),
          W_D(constants.W_D),
          ZETA(constants.ZETA),
          SYSTEM_DELAY_SEC(constants.SYSTEM_DELAY_SEC),
          TorqueToMotorOutput(constants.TorqueToMotorOutput)

    {
        calculateConstants();
    }

    float getOptimalTorque(float posError, float vel)
    {
        // Helpful to uncomment when tuning
        // calculateConstants();

        // Small linear interp to help with delay
        float xe = -posError + vel * SYSTEM_DELAY_SEC;

        // Quadrant 1: Overshot target, moving away = Max Brake
        if (xe > 0.0f && vel >= 0.0f) return -TAU_MAX * TorqueToMotorOutput;

        // Quadrant 3: Undershot target, moving away = Max Brake
        if (xe < 0.0f && vel <= 0.0f) return TAU_MAX * TorqueToMotorOutput;

        // Quadrant 4: Past target, moving back towards it
        if (xe >= 0.0f && vel < 0.0f)
        {
            return -evaluateF1(-xe, -vel);  // Symmetry property
        }

        // Quadrant 2: Behind target, moving towards it
        if (xe <= 0.0f && vel > 0.0f)
        {
            return evaluateF1(xe, vel);
        }

        return 0.0f;  // Exactly at target, zero velocity
    }

private:
    // Evaluates the IEEE Eq. 21 STOS Switching Logic for the 2nd Quadrant
    float evaluateF1(float xe, float v)
    {
        // in the little x_star region
        if (xe >= x_star_const && v <= S4_const)
        {
            return (-a2 * TAU_MAX * TorqueToMotorOutput);
        }

        if (v <= S1_const && xe < x_star_const)
        {
            return a1 * TAU_MAX * TorqueToMotorOutput;
        }

        // 1. Calculate the Dynamic S2 Curve (Boundary between +U and +a2_kick)
        float arg2 = 1.0f + (a * v - a2 * U_b2 * (1.0f - E_pos)) / (U_b2 * (a1 + a2 * E_pos));
        if (arg2 < 1e-6f) arg2 = 1e-6f;  // Safety clamp for log
        float S2_curve = (-v / a) + (U_b2 / (a * a)) * std::log(arg2) - (a2 * U_b2 / a) * T_d;

        // 2. Calculate the Dynamic S3 Curve (Boundary between +a2_kick and -U brake)
        float arg3 = 1.0f + (a * v - a1 * U_b2 * (E_neg - 1.0f)) / (U_b2 * (a1 * E_neg + a2));
        if (arg3 < 1e-6f) arg3 = 1e-6f;  // Safety clamp for log
        float S3_curve = (-v / a) + (U_b2 / (a * a)) * std::log(arg3) - (U_b2 / a) * T_d;

        // 3. The Geometric Evaluator (Left to Right on the Phase Plane)
        if (xe < S2_curve)
        {
            // Region 1 & 2: Accelerating
            return (v < S1_const) ? (a1 * TAU_MAX * TorqueToMotorOutput)
                                  : (TAU_MAX * TorqueToMotorOutput);
        }
        else if (xe < S3_curve)
        {
            // Region 3: The middle dip
            return (a2 - a1) * TAU_MAX * TorqueToMotorOutput;
        }
        else
        {
            // Region 4 & 5: Braking
            return (v > S4_const) ? (-TAU_MAX * TorqueToMotorOutput)
                                  : (-a2 * TAU_MAX * TorqueToMotorOutput);
        }
    }
    void calculateConstants()
    {
        a = B_DAMP / J_TOTAL;
        float b2 = 1 / J_TOTAL;
        U_b2 = TAU_MAX * b2;

        float M = std::exp(-(ZETA * M_PI) / std::sqrt(1.0f - (ZETA * ZETA)));
        a1 = 1.0f / (1.0f + M);
        a2 = M / (1.0f + M);
        T_d = static_cast<float>(M_PI) / W_D;

        E_pos = std::exp(a * T_d);
        E_neg = std::exp(-a * T_d);

        S1_const = (a1 * U_b2 / a) * (1.0f - E_neg);
        S4_const = (a2 * U_b2 / a) * (E_pos - 1.0f);

        x_star_const = (U_b2 / (a * a)) * std::log((a1 + a2 * E_pos) / (a2 + a1 * E_neg)) -
                       (U_b2 * a2 / (a * a)) * (E_pos - 1.0f) - (a1 * U_b2) / (a)*T_d;
    }
};
}  // namespace aruwsrc::control::turret::algorithms

#endif  // TURRET_STOS_CONTROLLER_HPP_