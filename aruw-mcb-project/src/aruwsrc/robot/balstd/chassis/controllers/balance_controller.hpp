#ifndef BALANCE_CONTROLLER_HPP_
#define BALANCE_CONTROLLER_HPP_

// #include "tap/algorithms/kalman_filter.hpp"

#include "tap/algorithms/smooth_pid.hpp"

#include "chassis_controller_interface.hpp"

namespace aruwsrc::control::balstd
{
class BalanceController : public BalstdChassisControllerInterface
{
public:
    float chassisWeight = 9 * 9.8;  // f = ma

    float heightSetpoint = 0.11;
    float heightSetpointTarget = 0.17;
    float heightSetpointRampRate = 0.05f / 2.0f / 500.0f;  // 0.05m / 5s / 500ticks/s=

    float rollSetpoint = 0;
    float yawSetpoint = 0;

    float LQRScalar = 0.5;
    float LQRWheelScalar = 1.0;
    float LQRHipScalar = 1.0;
    float gravityScalar = 0.55;
    float hipTorqueOverride = 0.1;

    BalanceController(
        const BalstdControlOperatorInterface& controlOperatorInterface,
        const tap::algorithms::SmoothPidConfig heightControllerConfig,
        const tap::algorithms::SmoothPidConfig splitControllerConfig,
        const tap::algorithms::SmoothPidConfig rollControllerConfig,
        const tap::algorithms::SmoothPidConfig yawControllerConfig)
        : BalstdChassisControllerInterface(controlOperatorInterface),
          heightController(heightControllerConfig),
          splitController(splitControllerConfig),
          rollController(rollControllerConfig),
          yawController(yawControllerConfig),
          vmState({{0, 0, 0, 0, 0, 0}}),
          vmRef({{0, 0, 0, 0, 0, 0}})
    {
    }

    BalstdChassisOutput runController(const BalstdChassisState& state, float dt) override;

    // void initialize() override
    // {
    //     heightSetpoint = 0.11;
    //     vmRef.data = {0, 0, 0, 0, 0, 0};
    // }

private:
    tap::algorithms::SmoothPid heightController, splitController, rollController, yawController;

    tap::algorithms::CMSISMat<6, 1> vmState, vmRef;

    // tap::algorithms::KalmanFilter<6, 6> stateFilter;

    tap::algorithms::CMSISMat<2, 6> getLQRGains(const float legLength) const;

    Vector vmLegForces(float hipTorque, float downwardForce, const BalstdLegState& currState) const;
};
}  // namespace aruwsrc::control::balstd

#endif  // BALANCE_CONTROLLER_HPP_