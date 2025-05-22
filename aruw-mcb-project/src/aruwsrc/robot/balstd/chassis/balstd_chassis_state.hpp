
#ifndef BALSTD_CHASSIS_STATE_HPP_
#define BALSTD_CHASSIS_STATE_HPP_

#include "tap/algorithms/transforms/transform.hpp"

#include "balstd_leg.hpp"

namespace aruwsrc::control::balstd
{

struct BalstdChassisState
{
    BalstdLegState leftLegState, rightLegState, virtualLegState;
    float roll, rollVel, pitch, pitchVel, yaw, yawVel;
    float height;
    float virtualPendTheta, virtualPendThetaDot;
    float virtualWheelPos, virtualWheelVel;  // x pos/vel of the 2d robot model's wheel
};

const BalstdChassisState ZERO_STATE{
    .leftLegState = BalstdLegState(),
    .rightLegState = BalstdLegState(),
    .virtualLegState = BalstdLegState(),
    .roll = 0,
    .rollVel = 0,
    .pitch = 0,
    .pitchVel = 0,
    .yaw = 0,
    .yawVel = 0,
    .height = 0,
    .virtualPendTheta = 0,
    .virtualPendThetaDot = 0,
    .virtualWheelPos = 0,
    .virtualWheelVel = 0,
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_CHASSIS_STATE_HPP_