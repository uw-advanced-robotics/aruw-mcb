#include "balance_controller.hpp"

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/math_user_utils.hpp"

using tap::algorithms::CMSISMat;

namespace aruwsrc::control::balstd
{

BalstdChassisOutput BalanceController::runController(const BalstdChassisState& currState, float dt)
{
    // LQR
    this->vmState.data = {
        currState.virtualPendTheta,
        currState.virtualPendThetaDot,
        currState.virtualWheelPos,
        currState.virtualWheelVel,
        currState.pitch,
        currState.pitchVel};
    CMSISMat<6, 1> vmRef = CMSISMat<6, 1>({0, 0, 0, 0, 0, 0});

    // u = K(x_d - x)
    CMSISMat<2, 1> vmOuts = getLQRGains(currState.virtualLegState.L) * (vmRef - vmState);
    float hipTorque = vmOuts.data[0] / 2;
    float wheelTorque = vmOuts.data[1] / 2;
    // virtual model has 1 hip/wheel, so we divide by 2 because we have 2

    float heightControllerOut =
        heightController.runControllerDerivateError(heightSetpoint - currState.height, dt);

    float splitControllerOut = splitController.runControllerDerivateError(
        currState.rightLegState.alpha - currState.leftLegState.alpha,
        dt);

    float rollControllerOut =
        rollController.runController(rollSetpoint - currState.roll, -currState.rollVel, dt);

    float yawControllerOut =
        yawController.runController(yawSetpoint - currState.yaw, -currState.yawVel, dt);

    float leftDownForce = chassisWeight + heightControllerOut + rollControllerOut;
    float rightDownForce = chassisWeight + heightControllerOut - rollControllerOut;

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

CMSISMat<2, 6> BalanceController::getLQRGains(const float) const
{
    // clang-format off
    return CMSISMat<2, 6>({
            -49.12, -8.4605, -20.491, -17.87, 31.272, 3.3703,
             33.962, 6.4306, 17.903, 14.759, 129.66, 6.5244
        });
    // clang-format on
}

}  // namespace aruwsrc::control::balstd
