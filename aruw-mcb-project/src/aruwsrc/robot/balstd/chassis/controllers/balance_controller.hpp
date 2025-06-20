#ifndef BALANCE_CONTROLLER_HPP_
#define BALANCE_CONTROLLER_HPP_

// #include "tap/algorithms/kalman_filter.hpp"

#include "tap/algorithms/ramp.hpp"
#include "tap/algorithms/smooth_pid.hpp"

#include "chassis_controller_interface.hpp"

namespace aruwsrc::control::balstd
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

        float minHeight, maxHeight;
    };

    BalanceController(
        const BalstdControlOperatorInterface& controlOperatorInterface,
        const Config config);

    void initialize(const BalstdChassisState& state) override;

    BalstdChassisOutput runController(const BalstdChassisState& state, float dt) override;

private:
    Config config;
    tap::algorithms::SmoothPid heightController, splitController, rollController, yawController;

    tap::algorithms::CMSISMat<6, 1> vmState, vmRef;

    // tap::algorithms::KalmanFilter<6, 6> stateFilter;

    tap::algorithms::CMSISMat<2, 6> getLQRGains(const float legLength) const;

    Vector vmLegForces(float hipTorque, float downwardForce, const BalstdLegState& currState) const;

    static constexpr float chassisWeight = 9 * 9.8;  // f = ma

    tap::algorithms::Ramp heightSetpoint;
    float rollSetpoint = 0;
    float yawSetpoint = 0;

    // this should all be const but isn't for ozonability
    float heightSetpointRampRate = 0.05f / 2.0f / 500.0f;  // 0.05m / 2s * 1s/500ticks
    float LQRScalar = 0.5;  // still no idea why everything has to be halved
    float LQRWheelScalar = 1.0;
    float LQRHipScalar = 1.0;
    float gravityScalar = 0.55;
};
}  // namespace aruwsrc::control::balstd

#endif  // BALANCE_CONTROLLER_HPP_