#include "balance_controller.hpp"

#include "tap/algorithms/cmsis_mat.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/wrapped_float.hpp"

using tap::algorithms::Angle;
using tap::algorithms::CMSISMat;
using tap::algorithms::WrappedFloat;

namespace aruwsrc::control::balstd
{

BalstdChassisOutput BalanceController::runController(const BalstdChassisState& currState, float dt)
{
    // LQR
    this->vmState.data = {
        -currState.virtualPendTheta,
        -currState.virtualPendThetaDot,
        currState.virtualWheelPos,
        currState.virtualWheelVel,
        currState.pitch,
        currState.pitchVel};
    CMSISMat<6, 1> vmRef = CMSISMat<6, 1>({0, 0, 0, 0, 0, 0});

    // u = K(x_d - x)
    CMSISMat<2, 1> vmOuts = getLQRGains(currState.virtualLegState.L) * (vmRef - vmState);
    float hipTorque = -vmOuts.data[0] / 2 * LQRHipScalar;
    // hipTorque = hipTorqueOverride;
    float wheelTorque = vmOuts.data[1] / 2 * LQRWheelScalar;
    // virtual model has 1 hip/wheel, so we divide by 2 because we have 2

    float heightControllerOut =
        heightController.runControllerDerivateError(currState.height - heightSetpoint, dt);

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

CMSISMat<2, 6> BalanceController::getLQRGains(const float) const
{
    // clang-format off
    return CMSISMat<2, 6>({
              -26.87 ,    -3.1818 ,     -16.432  ,    -13.545 ,      54.018   ,    6.4121,
       32.997    ,    4.693      , 30.331  ,     23.121   ,    100.92   ,     4.841,
           }) * LQRScalar;
    // clang-format on
}

}  // namespace aruwsrc::control::balstd
