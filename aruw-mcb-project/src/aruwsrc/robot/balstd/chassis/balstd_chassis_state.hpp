
#ifndef BALSTD_CHASSIS_STATE_HPP_
#define BALSTD_CHASSIS_STATE_HPP_

#include "tap/algorithms/transforms/transform.hpp"

#include "balstd_leg.hpp"

namespace aruwsrc::control::balstd
{

struct BalstdChassisState
{
    BalstdLegState leftLegState, rightLegState, virtualLegState;
    float roll, pitch, yaw, height;
};

const BalstdChassisState ZERO_STATE{
    .leftLegState = BalstdLegState(),
    .rightLegState = BalstdLegState(),
    .virtualLegState = BalstdLegState(),
    .roll = 0,
    .pitch = 0,
    .yaw = 0,
    .height = 0,
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_CHASSIS_STATE_HPP_