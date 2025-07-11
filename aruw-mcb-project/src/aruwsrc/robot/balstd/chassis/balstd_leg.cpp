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

#include "balstd_leg.hpp"

using aruwsrc::control::motor::Tmotor_AK809;
using tap::algorithms::CMSISMat;

// Link Convention

/*           *  P3
 *        ⟋    ⟍
 *     ⟋          ⟍
 * P1 *             * P2
 *     \           /
 *      \         /
 *    P5 * ───── * P1
 *
 *  y
 *  ^
 *  |
 *    ──> x
 */

namespace aruwsrc::balstd::chassis
{
void BalstdLeg::initialize()
{
    frontHipMotor.initialize();
    backHipMotor.initialize();
    wheelMotor.initialize();
}

void BalstdLeg::refresh()
{
    frontHipMotor.sendCanMessage();
    backHipMotor.sendCanMessage();
}

bool BalstdLeg::allMotorsOnline() const
{
    return frontHipMotor.isMotorOnline() && backHipMotor.isMotorOnline() &&
           wheelMotor.isMotorOnline();
}

void BalstdLeg::setThrust(const Vector thrust)
{
    CMSISMat<2, 1> torques = currState.jacobianT * CMSISMat<2, 1>({thrust.x(), thrust.y()});

    setHipTorques(torques.data[0], torques.data[1]);
}

void BalstdLeg::setWheelTorque(float torque)
{
    wheelMotor.setDesiredOutput(torque * M3508_TORQUE_CONSTANT);
}

void BalstdLeg::setHipTorques(float front, float back)
{
    // soft stops
    if (currState.qFront <= config.frontHipOuterLimit && front > 0) front = 0;
    if (currState.qFront >= config.frontHipInnerLimit && front < 0) front = 0;

    if (currState.qBack <= config.backHipInnerLimit && back > 0) back = 0;
    if (currState.qBack >= config.backHipOuterLimit && back < 0) back = 0;

    frontHipMotor.setDesiredOutput(
        tap::algorithms::limitVal(front * Tmotor_AK809::TORQUE_CONSTANT, -15000.0f, 15000.0f));
    backHipMotor.setDesiredOutput(
        tap::algorithms::limitVal(back * Tmotor_AK809::TORQUE_CONSTANT, -15000.0f, 15000.0f));
}

void BalstdLeg::updateState()
{
    currState.qFront = getFrontHipAngle();
    currState.qBack = getBackHipAngle();

    // TODO: why negative
    currState.qFrontVelo = -frontHipMotor.getEncoder()->getVelocity();
    currState.qBackVelo = -backHipMotor.getEncoder()->getVelocity();

    currState.wheelVel = wheelMotor.getEncoder()->getVelocity();
    currState.calculateForwardKinematics(config);

    currState.calculateJacobian(config);
    currState.calculateWheelTranslationVelocity();
    currState.calculatePendulumState();
}

std::array<float, 2> BalstdLegState::calculateLegEnergy(BalstdLegConfig config)
{
    // 1/2 I * w^2 Upper link energy
    float Ta = .5 * config.frontUpperLegLinkInertia * (qFrontVelo) * (qFrontVelo);

    // Rotational Energy of lower link
    float Tb_t = .5 * config.frontLowerLegLinkInertia * (qLowerFrontVelo) * (qLowerFrontVelo);
    CMSISMat<3, 1> VP2 = cross({{0.0f, 0.0f, qFrontVelo}}, (P2 - P1));

    // Velocity of point halfway along the leg linkage for translational energy calculation
    CMSISMat<3, 1> VP3_2 = cross({{0.0f, 0.0f, qLowerFrontVelo}}, (P3 - P2) / 2.0f) + VP2;

    // Translational energy of lower link
    float Tb_x = .5 * config.backLowerLegLinkInertia *
                 (VP3_2.data[0] * VP3_2.data[0] + VP3_2.data[1] * VP3_2.data[1]);

    // Translational energy of the wheel
    float Tw = .5 * config.wheelMass * sqrtf(vxc * vxc + vyc * vyc);

    float energy = Ta + Tb_t + Tb_x + Tw / 2;

    return {energy, 0};
}

float BalstdLeg::updateCBF(BalstdLegState leg)
{
    // Check if we're outside the limits, control within since the CBF is gonna go negative
    if (currState.qFront <= config.frontHipOuterLimit)
    {
        // We have hit the hardstop move out of it as much as possible
        return maxTorque;
    }
    if (currState.qFront >= config.frontHipInnerLimit)
    {
        return -maxTorque;
    }
    if (currState.qBack >= config.backHipOuterLimit)
    {
        return maxTorque;
    }
    if (currState.qBack <= config.frontHipInnerLimit)
    {
        return -maxTorque;
    }
    // calculate the available torque from the motors
    // assume that gravity is the only force acting on the end effector
    CMSISMat<2, 1> endEffectorTorque =
        currState.jacobianT * CMSISMat<2, 1>({0, 9.8f * config.balstdMass / 4});

    // Right half of the linkage (per the paper orientation)
    float torque_available = maxTorque - endEffectorTorque.data[0];
    float distance_to_stop = currState.qFront - config.frontHipOuterLimit;

    /*
       this makes the incorrect assumption that the available torque will be constant
       through the travel. Doing an integration of the required torque would be better, something
       to do later.
    */
    float availableEnergy = (torque_available * distance_to_stop);

    /*
       In this system, the energy is from each link and the wheel translation, coming to
       Ta + Tb + Tc + Td + Tw, however we only are going to consider the energy from the
       corresponding half of the link coming to Ta + Tb + 1/2 * Tw.
    */
    std::array<float, 2> energy = leg.calculateLegEnergy(config);

    if (energy[0] - CBF_ENERGY_LIMIT > availableEnergy)
    {
        return maxTorque;
    }
    return 0;  // TODO: should pass though requested torque, this method structure should be
               // rethought
}
}  // namespace aruwsrc::balstd::chassis