#include "balstd_leg_hardstop_cbf.hpp"

#include "tap/algorithms/cmsis_mat.hpp"

using tap::algorithms::CMSISMat;

namespace aruwsrc::control::balstd
{
LegHardstopCBF::LegHardstopCBF(
    tap::algorithms::CMSISMat<2, 2>& jacobianTranspose,
    BalstdLegState& LegState,
    const BalstdLegConfig config,
    float energy_limit,
    float maxTorque)
    : jacobianTranspose(jacobianTranspose),
      invjacobianTranspose(jacobianTranspose.inverse()),
      LegState(LegState),
      config(config),
      energy_limit(energy_limit),
      maxTorque(maxTorque)
{
}

float LegHardstopCBF::update()
{

    // calculate the available torque from the motors
    // assume that gravity is the only force acting on the end effector
    CMSISMat<2, 1> endEffectorTorque = invjacobianTranspose * CMSISMat<2, 1>({0, 9.8 * BALSTDWEIGHT / 4});

    float torque_available = maxTorque - endEffectorTorque.data[0];
    float distance_to_stop = LegState.qFront - config.frontHipOuterLimit;

    /* this makes the incorrect assumption that the available torque will be constant
       through the travel. Doing an integration of the required torque would be better, something
       to do later. 
    */
    float availableEnergy = (torque_available * distance_to_stop);

    /*
       In this system, the energy is from each link and the wheel translation, coming to
       Ta + Tb + Tc + Td + Tw, however we only are going to consider the energy from the corresponding
       half of the link coming to Ta + Tb + 1/2 * Tw.
    */ 


    // 1/2 I * w^2 (PROBABLY REMOVE LATER BUT NOTE TO SELF, BECAUSE THE CHASSIS MOVES WITH THE LEG THERE ISN'T TRANSLATIONAL ENERGY TO KILL HERE)
    float Ta = .5 * upper_link_inertia * (LegState.qFrontVelo)*(LegState.qFrontVelo);
    float Tb = .5 * lower_link_inertia * (LegState.qLowerFrontVelo)*(LegState.qLowerFrontVelo) + .5 * lower_link_mass * 
    (config.upperLinkLength * config.upperLinkLength * LegState.qFrontVelo*LegState.qFrontVelo + config.lowerLinkLength*config.lowerLinkLength *LegState.qLowerFrontVelo * LegState.qLowerFrontVelo * .25
     + config.upperLinkLength*config.lowerLinkLength * .5 * LegState.qFrontVelo * LegState.qLowerFrontVelo * cos(LegState.qFront - LegState.qLowerFront));

    float Tw = .5 * wheel_mass * LegState.xcVelo * LegState.xcVelo + .5 * wheel_mass * LegState.ycVelo * LegState.ycVelo;

    float energy = Ta + Tb + Tw/2;

    if (energy - energy_limit > availableEnergy){
        return 1000000000000.0f;
    }

    
}

}  // namespace aruwsrc::control::balstd
