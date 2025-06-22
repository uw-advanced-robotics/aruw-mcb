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

#include "balance_controller.hpp"

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/wrapped_float.hpp"

#include "modm/architecture/utils.hpp"

using tap::algorithms::Angle;
using tap::algorithms::CMSISMat;
using tap::algorithms::WrappedFloat;

namespace aruwsrc::balstd::chassis::controllers
{

BalanceController::BalanceController(
    const BalstdControlOperatorInterface& controlOperatorInterface,
    const Config config)
    : BalstdChassisControllerInterface(controlOperatorInterface),
      config(config),
      heightController(config.heightControllerConfig),
      splitController(config.splitControllerConfig),
      rollController(config.rollControllerConfig),
      yawController(config.yawControllerConfig),
      vmState({{0, 0, 0, 0, 0, 0}}),
      vmRef({{0, 0, 0, 0, 0, 0}}),
      heightSetpoint(config.minHeight)
{
    heightSetpoint.setTarget(0.17);
}

void BalanceController::initialize(const BalstdChassisState& state)
{
    heightSetpoint.setValue(state.height);
    vmRef.data = {0, 0, state.virtualWheelPos, 0, 0, 0};
}

BalstdChassisOutput BalanceController::runController(const BalstdChassisState& currState, float dt)
{
    // update state references
    vmRef.data[2] += controlOperatorInterface.getXVel() * dt;
    yawSetpoint += controlOperatorInterface.getYawVel() * dt;

    heightSetpoint.setTarget(
        std::clamp(
            heightSetpoint.getTarget() + controlOperatorInterface.getHeightVel() * dt,
            config.minHeight,
            config.maxHeight));
    heightSetpoint.update(heightSetpointRampRate);

    // LQR
    this->vmState.data = {
        currState.virtualPendTheta,
        currState.virtualPendThetaDot,
        currState.virtualWheelPos,
        currState.virtualWheelVel,
        -currState.pitch,
        -currState.pitchVel};

    // u = K(x_d - x)
    CMSISMat<2, 1> vmOuts = getLQRGains(currState.virtualLegState.L) * (vmRef - vmState);
    float hipTorque = -vmOuts.data[1] / 2 * LQRHipScalar;
    float wheelTorque = vmOuts.data[0] / 2 * LQRWheelScalar;
    // virtual model has 1 hip/wheel, so we divide by 2 because we have 2

    float heightControllerOut = heightController.runControllerDerivateError(
        currState.height - heightSetpoint.getValue(),
        dt);

    float splitControllerOut = splitController.runController(
        currState.leftLegState.alpha - currState.rightLegState.alpha,
        currState.rightLegState.alphaDot - currState.leftLegState.alphaDot,
        dt);

    float rollControllerOut =
        rollController.runController(currState.roll - rollSetpoint, -currState.rollVel, dt);

    float yawControllerOut = yawController.runController(
        Angle(currState.yaw).minDifference(yawSetpoint),
        -currState.yawVel,
        dt);

    float leftDownForce = -chassisWeight * gravityScalar + heightControllerOut + rollControllerOut;
    float rightDownForce = -chassisWeight * gravityScalar + heightControllerOut - rollControllerOut;

    float leftHipTorque = hipTorque + splitControllerOut;
    float rightHipTorque = hipTorque - splitControllerOut;

    float leftWheelTorque = wheelTorque - yawControllerOut;
    float rightWheelTorque = wheelTorque + yawControllerOut;

    return BalstdChassisOutput(
        vmLegForces(leftHipTorque, leftDownForce, currState.leftLegState),
        vmLegForces(rightHipTorque, rightDownForce, currState.rightLegState),
        leftWheelTorque,
        rightWheelTorque);
}

Vector BalanceController::vmLegForces(
    float hipTorque,
    float downwardForce,
    const BalstdLegState& currState) const
{
    float horizontalForce = -hipTorque / currState.L;
    tap::algorithms::rotateVector(&horizontalForce, &downwardForce, currState.alpha);
    return Vector(horizontalForce, downwardForce, 0);
}

CMSISMat<2, 6> BalanceController::getLQRGains(const float legLength) const
{
    // TODO: use config for all this stuff
    // clang-format off
    // return CMSISMat<2, 6>({  // for length = 0.17m
    //    -37.831, -5.031, -20.811, -17.182, 31.92, 5.0632,
    //    12.604, 1.6434, 8.1786, 6.2358, 68.298, 6.4245
    //        }) * LQRScalar;
    
    float coeffs[5][12] {
      {-9.0447, -1.6074, -16.313, -14.196,  61.599,  9.5572,
        6.3627, 0.92331,  15.717,  11.688,  49.341,  3.1599},
      {-204.39, -8.0768, -47.927, -15.125, -261.93, -45.14,
        85.808,  6.4024, -60.963, -47.643,  206.25,  35.747},
      { 247.44, -100.06,  171.53, -37.581,  638.76,  151.36,
       -425.02, -16.212,  106.72,  113.15, -769.64, -136.04},
      {-261.11,  199.68, -302.23,  169.06, -800.04, -279.79,
         936.1,  22.581, -42.063, -137.25,  1429.5,  262.33},
      { 111.38, -166.65,  212.08, -188.46,  387.98,  214.83,
        -788.7,  -12.93, -59.553,  62.965, -1061.1, -202.77}};
    // clang-format on

    CMSISMat<2, 6> K(coeffs[0]);
    float x = legLength;
    for (unsigned int i = 1; i < MODM_ARRAY_SIZE(coeffs); i++, x *= legLength)
    {
        for (int j = 0; j < 12; j++)
        {
            K.data[j] += coeffs[i][j] * x;
        }
    }

    return K;
}

}  // namespace aruwsrc::balstd::chassis::controllers
