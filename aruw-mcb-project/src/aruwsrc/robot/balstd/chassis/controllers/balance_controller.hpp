/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALANCE_CONTROLLER_HPP_
#define BALANCE_CONTROLLER_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/algorithms/smooth_pid.hpp"

#include "chassis_controller_interface.hpp"

namespace aruwsrc::balstd::chassis::controllers
{
class BalanceController : public BalstdChassisControllerInterface
{
public:
    struct Config
    {
        tap::algorithms::SmoothPidConfig heightControllerConfig;
        tap::algorithms::SmoothPidConfig splitControllerConfig;
        tap::algorithms::SmoothPidConfig rollControllerConfig;
        tap::algorithms::SmoothPidConfig yawControllerConfig;

        float minHeight, maxHeight;  // m
        float maxHeightSetpointVel;  // m/s
        float maxRollSetpointVel;    // rad/s
    };

    BalanceController(
        BalstdControlOperatorInterface& controlOperatorInterface,
        const Config config);

    void initialize(const BalstdChassisState& state) override;

    BalstdChassisOutput runController(const BalstdChassisState& state, float dt) override;

    tap::algorithms::CMSISMat<2, 6> getLQRGains(const float legLength) const;

#if not defined(PLATFORM_HOSTED) || not defined(ENV_UNIT_TESTS)
private:
#endif
    const Config config;
    tap::algorithms::SmoothPid heightController, splitController, rollController, yawController;

    tap::algorithms::CMSISMat<6, 1> vmState, vmRef;

    tap::algorithms::Ramp heightSetpoint;
    tap::algorithms::Ramp rollSetpoint;
    float yawSetpoint = 0;

    Vector vmLegForces(float hipTorque, float downwardForce, const BalstdLegState& currState) const;

    // this should all be const but isn't for ozonability
    static constexpr float chassisWeight = 9 * 9.8;  // f = ma
    float LQRScalar = 0.5;  // still no idea why everything has to be halved
    float LQRWheelScalar = 1.0;
    float LQRHipScalar = 1.0;
    float gravityScalar = 0.5f;
};
}  // namespace aruwsrc::balstd::chassis::controllers

#endif  // BALANCE_CONTROLLER_HPP_