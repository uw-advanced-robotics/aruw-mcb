
#ifndef BALSTD_CHASSIS_STATE_HPP_
#define BALSTD_CHASSIS_STATE_HPP_

#include "tap/algorithms/transforms/transform.hpp"

#include "balstd_leg.hpp"

namespace aruwsrc::control::balstd
{

struct BalstdChassisState
{
    BalstdLegState leftLegState, rightLegState, virtualLegState;
    float chassisPitch;
};

const BalstdChassisState ZERO_STATE{
    .leftLegState = {0, 0, 0, 0, 0, 0},
    .rightLegState = {0, 0, 0, 0, 0, 0},
    .virtualLegState = {0, 0, 0, 0, 0, 0},
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_CHASSIS_STATE_HPP_