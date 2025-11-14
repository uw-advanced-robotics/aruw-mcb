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
using Input = aruwsrc::balstd::BalstdControlOperatorInterface::Input;

namespace aruwsrc::balstd::chassis::controllers
{
BalanceController::BalanceController(
    BalstdControlOperatorInterface& controlOperatorInterface,
    const Config config)
    : BalstdChassisControllerInterface(controlOperatorInterface),
      config(config),
      heightController(config.heightControllerConfig),
      splitController(config.splitControllerConfig),
      rollController(config.rollControllerConfig),
      yawController(config.yawControllerConfig),
      vmState({{0, 0, 0, 0, 0, 0}}),
      vmRef({{0, 0, 0, 0, 0, 0}}),
      heightSetpoint(config.minHeight),
      rollSetpoint(0)
{
}

void BalanceController::initialize(const BalstdChassisState& state)
{
    heightSetpoint.setTarget(state.height);  // 0.17
    heightSetpoint.setValue(state.height);
    yawSetpoint = state.yaw;
    vmRef.data = {0, 0, state.virtualWheelPos, 0, 0, 0};
    controlOperatorInterface.setMode(BalstdControlOperatorInterface::Mode::BALANCE);
}

float leftHipTorque = 0;
float rightHipTorque = 0;
float hipTorque;

BalstdChassisOutput BalanceController::runController(const BalstdChassisState& currState, float dt)
{
    // update state references
    vmRef.data[2] += controlOperatorInterface.getInput<Input::X_VEL>() * dt;
    yawSetpoint += controlOperatorInterface.getInput<Input::YAW_VEL>() * dt;

    heightSetpoint.setTarget(
        std::clamp(
            heightSetpoint.getTarget() +
                controlOperatorInterface.getInput<Input::HEIGHT_VEL>() * dt,
            config.minHeight,
            config.maxHeight));
    heightSetpoint.update(config.maxHeightSetpointVel * dt);

    rollSetpoint.setTarget(controlOperatorInterface.getInput<Input::ROLL>());
    rollSetpoint.update(config.maxRollSetpointVel * dt);

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
    hipTorque = -vmOuts.data[1] / 2 * LQRHipScalar;
    float wheelTorque = vmOuts.data[0] / 2 * LQRWheelScalar;
    // virtual model has 1 hip/wheel, so we divide by 2 because we have 2

    float heightControllerOut = heightController.runControllerDerivateError(
        currState.height - heightSetpoint.getValue(),
        dt);

    float splitControllerOut = splitController.runController(
        currState.leftLegState.alpha - currState.rightLegState.alpha,
        currState.rightLegState.alphaDot - currState.leftLegState.alphaDot,
        dt);

    float rollControllerOut = rollController.runController(
        currState.roll - rollSetpoint.getValue(),
        -currState.rollVel,
        dt);

    float yawControllerOut = yawController.runController(
        Angle(currState.yaw).minDifference(yawSetpoint),
        -currState.yawVel,
        dt);

    float leftDownForce = -chassisWeight * gravityScalar + heightControllerOut + rollControllerOut;
    float rightDownForce = -chassisWeight * gravityScalar + heightControllerOut - rollControllerOut;

    leftHipTorque = hipTorque + splitControllerOut;
    rightHipTorque = hipTorque - splitControllerOut;

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
    float tangentForce = -hipTorque / currState.L;
    float radialForce = 0;
    tap::algorithms::rotateVector(&tangentForce, &radialForce, currState.alpha);
    return Vector(tangentForce, downwardForce + radialForce, 0);
}

CMSISMat<2, 6> BalanceController::getLQRGains(const float legLength) const
{
    // TODO: use config for all this stuff
    // clang-format off
    // float coeffs[1][12] {{  // for length = 0.17m
    //    -37.831, -5.031, -20.811, -17.182, 31.92, 5.0632,
    //    12.604, 1.6434, 8.1786, 6.2358, 68.298, 6.4245}};
    
    float coeffs[5][12] = {
      {-9.85892, -1.71311, -22.4561, -16.5944, 63.9553, 10.1073, 
        6.99959, 0.987261, 22.9386, 13.8334, 47.3585, 2.7125},
      {-211.55, -8.72232, -72.7963, -15.5699, -272.821, -47.9073, 
        91.9895, 7.50419, -89.233, -56.5596, 222.353, 39.3458},
      {191.523, -114.233, 262.476, -72.6831, 686.252, 165.399, 
        -444.662, -17.5943, 161.712, 142.337, -835.433, -151.229},
      {-135.821, 221.321, -467.118, 269.771, -908.385, -313.169, 
        967.834, 23.1256, -82.1246, -192.054, 1564.91, 294.532},
      {4.62621, -183.577, 331.555, -284.971, 484.149, 244.857, 
        -809.777, -12.3238, -64.7886, 105.893, -1171.68, -229.68},
    };
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

    return K * LQRScalar;
}

}  // namespace aruwsrc::balstd::chassis::controllers
