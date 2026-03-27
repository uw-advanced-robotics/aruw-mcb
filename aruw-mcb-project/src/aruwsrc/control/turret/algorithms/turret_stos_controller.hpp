#ifndef TURRET_STOS_CONTROLLER_HPP_
#define TURRET_STOS_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

#include "modm/math/geometry.hpp"

// See this paper: https://ieeexplore.ieee.org/document/1264127
namespace aruwsrc::control::turret::algorithms
{
class OptimalSTOSController
{
private:
    float a, U_b2;
    float a1, a2, T_d;
    float E_pos, E_neg;
    float S1_const, S4_const;

    float J_TOTAL;
    float TAU_MAX;
    float B_DAMP;
    float W_D;
    float ZETA;
    float SYSTEM_DELAY_SEC;

public:
    struct STOSConstants
    {
        float J_TOTAL;
        float TAU_MAX;
        float B_DAMP;
        float W_D;
        float ZETA;
        float SYSTEM_DELAY_SEC;
    };

    OptimalSTOSController(STOSConstants constants)
        : J_TOTAL(constants.J_TOTAL),
          TAU_MAX(constants.TAU_MAX),
          B_DAMP(constants.B_DAMP),
          W_D(constants.W_D),
          ZETA(constants.ZETA),
          SYSTEM_DELAY_SEC(constants.SYSTEM_DELAY_SEC)
    {
        calculateConstants();
    }

    float getOptimalTorque(float posError, float vel)
    {
        // Helpful to uncomment when tuning
        // calculateConstants();

        // Small linear interp to help with delay
        float xe = -posError - vel * SYSTEM_DELAY_SEC;

        // Quadrant 1: Overshot target, moving away = Max Brake
        if (xe > 0.0f && vel >= 0.0f) return -TAU_MAX;

        // Quadrant 3: Undershot target, moving away = Max Brake
        if (xe < 0.0f && vel <= 0.0f) return TAU_MAX;

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
            return (v < S1_const) ? (a1 * TAU_MAX) : TAU_MAX;
        }
        else if (xe < S3_curve)
        {
            // Region 3: The middle dip
            return (a2 - a1) * TAU_MAX;
        }
        else
        {
            // Region 4 & 5: Braking
            return (v > S4_const) ? -TAU_MAX : (-a2 * TAU_MAX);
        }
    }
    void calculateConstants()
    {
        a = B_DAMP / J_TOTAL;
        U_b2 = TAU_MAX / J_TOTAL;

        float denom = std::sqrt(1.0f - (ZETA * ZETA));
        float M = std::exp(-(ZETA * static_cast<float>(M_PI)) / denom);
        a1 = 1.0f / (1.0f + M);
        a2 = M / (1.0f + M);
        T_d = static_cast<float>(M_PI) / W_D;

        E_pos = std::exp(a * T_d);
        E_neg = std::exp(-a * T_d);

        S1_const = (a1 * U_b2 / a) * (1.0f - E_neg);
        S4_const = (a2 * U_b2 / a) * (E_pos - 1.0f);
    }
};
}  // namespace aruwsrc::control::turret::algorithms

#endif  // TURRET_STOS_CONTROLLER_HPP_