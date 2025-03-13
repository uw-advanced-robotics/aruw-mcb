
#ifndef BALSTD_CHASSIS_OUTPUT_HPP_
#define BALSTD_CHASSIS_OUTPUT_HPP_

#include "tap/algorithms/transforms/vector.hpp"

#include "balstd_leg.hpp"

namespace aruwsrc::control::balstd
{

struct BalstdChassisOutput
{
    tap::algorithms::transforms::Vector leftForce, rightForce;
    float leftTorque, rightTorque;

    BalstdChassisOutput(float flx, float fly, float frx, float fry, float tl, float tr)
        : leftForce(flx, fly, 0.0f),
          rightForce(frx, fry, 0.0f),
          leftTorque(tl),
          rightTorque(tr)
    {
    }
};

const BalstdChassisOutput ZERO_OUTPUT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_CHASSIS_OUTPUT_HPP_