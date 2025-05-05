#ifndef BALSTD_LEG_HARDSTOP_CBF_HPP_
#define BALSTD_LEG_HARDSTOP_CBF_HPP_

#include "tap/algorithms/cmsis_mat.hpp"

#include "balstd_leg.hpp"


/**
 * The goal of this controller is to ensure that the leg
 * does not exceed soft stop limits. This is done via a
 * Control Barrier Function (CBF) that calculates the
 * system energy and ensures that the motor always has
 * enough torque and distance to stop before hitting the limit.
 *
 * This system only consideres the force on the end effector
 * applied by gravity and the corresponding half of the five
 * bar linkage's inertia.
 */

namespace aruwsrc::control::balstd
{
class LegHardstopCBF
{
public:
    /**
     * Constructor for the LegHardstopCBF class.
     * @param[in] jacobianTranspose The jacobian matrix used for the .
     * @param[in] LegState The state of the leg
     * @param[in] config The configuration of the leg.
     * @param[in] energy_limit The limit in energy allowed to hit the hardstop, set to 0 to not
     * allow any.
     * @param[in] maxTorque The maximum torque allowed to be applied to the motors.
     */
    LegHardstopCBF(
        tap::algorithms::CMSISMat<2, 2>& jacobianTranspose,
        BalstdLegState& LegState,
        const BalstdLegConfig config,
        float energy_limit,
        float maxTorque);

    float update();  // returns the torque to apply to the motors

private:
    tap::algorithms::CMSISMat<2, 2>& jacobianTranspose;
    tap::algorithms::CMSISMat<2, 2> invjacobianTranspose;
    BalstdLegState& LegState;
    const BalstdLegConfig config;
    float energy_limit;
    float maxTorque;

    float upper_link_inertia = 0.5;  // kg*m^2, TODO: get this from the robot config
    float lower_link_inertia = 0.5;  // kg*m^2, TODO: get this from the robot config
    float lower_link_mass = 0.5;  // kg, TODO: get this from the robot config
    float wheel_mass = 0.5;  // kg, TODO: get this from the robot config

    constexpr static float BALSTDWEIGHT = 19.5;  // kg, TODO: get this from the robot config
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_LEG_HARDSTOP_CBF_HPP_