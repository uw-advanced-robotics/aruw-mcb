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

#ifndef BALSTD_LEG_HPP_
#define BALSTD_LEG_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/transforms/vector.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/motor/tmotor_ak80_9.hpp"
#include "modm/math/geometry/angle.hpp"

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
using tap::algorithms::CMSISMat;
using tap::algorithms::transforms::Vector;
struct BalstdLegConfig
{
    float upperLinkLength;  // meters
    float lowerLinkLength;  // meters
    float fixedLinkLength;  // meters

    float frontHipOuterLimit;  // radians
    float frontHipInnerLimit;  // radians
    float backHipOuterLimit;   // radians
    float backHipInnerLimit;   // radians

    float frontUpperLegLinkInertia;  // kg*m^2
    float backUpperLegLinkInertia;   // kg*m^2
    float frontLowerLegLinkInertia;  // kg*m^2
    float backLowerLegLinkInertia;   // kg*m^2

    float frontUpperLegLinkMass;  // kg
    float backUpperLegLinkMass;   // kg
    float frontLowerLegLinkMass;  // kg
    float backLowerLegLinkMass;   // kg

    float wheelMass;  // kg

    float balstdMass;  // kg
};
struct BalstdLegState
{
    float qFront, qBack;                    // angles of upper linkages in radians
    float qFrontVelo, qBackVelo;            // velocities of upper linkages in rad/s
    float qLowerFront, qLowerBack;          // Angles of the lower linkages
    float qLowerFrontVelo, qLowerBackVelo;  // velocities of lower linkages in rad/s
    float wheelVel;                         // wheel angular velocity in rad/s

    CMSISMat<3, 1> P1, P2, P3, P4, P5;  // Positions of joints (x, y, z)

    float vxc, vyc;  // Velocity of the wheel transitionally

    float kneesWidthX, kneesWidthY;  // components of distance between knees

    float L, alpha, alphaDot;  // pendulum length and angle wrt hip center

    tap::algorithms::CMSISMat<2, 2> jacobianTranspose;

    void calculateJacobian(BalstdLegConfig config)
    {
        float p1x2 = -config.upperLinkLength * sin(qFront);
        float p1y2 = config.upperLinkLength * cos(qFront);
        float p5x4 = -config.upperLinkLength * sin(qBack);
        float p5y4 = config.upperLinkLength * cos(qBack);

        float d = sqrt(kneesWidthX * kneesWidthX + kneesWidthY * kneesWidthY);
        float h = sqrt(config.lowerLinkLength * config.lowerLinkLength - d * d / 4);

        float p1d = -(kneesWidthX * p1x2 + kneesWidthY * p1y2) / d;
        float p5d = (kneesWidthX * p5x4 + kneesWidthY * p5y4) / d;
        float p1h = -d * p1d / h / 4;
        float p5h = -d * p5d / h / 4;

        float p1x3 = p1x2 / 2 - h / d * p1y2 + (p1h * d - p1d * h) / (d * d) * kneesWidthY;
        float p1y3 = p1y2 / 2 + h / d * p1x2 - (p1h * d - p1d * h) / (d * d) * kneesWidthX;
        float p5x3 = p5x4 / 2 + h / d * p5y4 + (p5h * d - p5d * h) / (d * d) * kneesWidthY;
        float p5y3 = p5y4 / 2 - h / d * p5x4 - (p5h * d - p5d * h) / (d * d) * kneesWidthX;

        jacobianTranspose = CMSISMat<2, 2>({p1x3, p1y3, p5x3, p5y3});
    }

    void calculateForwardKinematics(BalstdLegConfig config)
    {
        // knee coordinates
        P2 = CMSISMat<3, 1>(
            {config.upperLinkLength * cos(qFront) + config.fixedLinkLength / 2,
             config.upperLinkLength * sin(qFront),
             0});

        P4 = CMSISMat<3, 1>(
            {config.upperLinkLength * cos(qBack) - config.fixedLinkLength / 2,
             config.upperLinkLength * sin(qBack),
             0});

        // wheel coordinates
        // ||P2-Ph|| = (a2^2 - a3^2 + ||P4-P2||^2) / (2*||P4-P2||)
        // a2 and a3 are the upper leg links and are the same
        // P4-P2
        CMSISMat<3, 1> P4_P2 = P4 - P2;

        kneesWidthX = P4_P2.data[0];
        kneesWidthY = P4_P2.data[1];

        // ||P4-P2||
        float P4_P2_mag = sqrtf(P4_P2.data[0] * P4_P2.data[0] + P4_P2.data[1] * P4_P2.data[1]);

        // Ph is the intersection point of the line that forms the knee and a perpendicular line to
        // the wheel

        // ||P2-Ph||
        float P2_Ph_mag = P4_P2_mag / 2;

        // Ph = P2 + ||P2-Ph|| / ||P2-P4|| * (P4-P2)
        CMSISMat<3, 1> Ph = P2 + P4_P2 / 2;

        // ||P3-Ph|| = sqrt(a2^2 - ||P2-Ph||^2)
        float P3_Ph_mag =
            sqrtf(config.lowerLinkLength * config.lowerLinkLength - P2_Ph_mag * P2_Ph_mag);

        // P3 = Ph ± ||P3-Ph|| / ||P2-P4|| * (P4-P2)
        CMSISMat<3, 1> P3_2 =
            Ph + P3_Ph_mag / P4_P2_mag * CMSISMat<3, 1>({P4_P2.data[1], -P4_P2.data[0], 0});

        P3 = {{P3_2.data[0], P3_2.data[1], 0}};
    }

    void calculatePendulumState()
    {
        alpha = atan2(-P3.data[0], P3.data[1]);
        alphaDot = tap::algorithms::cross({{vxc, vyc, 0.0f}}, P3).data[2];
        L = sqrt(P3.data[0] * P3.data[0] + P3.data[1] * P3.data[1]);
    }

    void calculateWheelTranslationVelocity()
    {
        const CMSISMat<2, 1> wheel_velo =
            jacobianTranspose.transpose() * CMSISMat<2, 1>({qFrontVelo, qBackVelo});
        vxc = wheel_velo.data[0];
        vyc = wheel_velo.data[1];
    }

    void calculateLowerLegVelocity(BalstdLegConfig config)
    {
        // ||P2 - P3|| / l
        CMSISMat<3, 1> VP2 = tap::algorithms::cross({{0.0f, 0.0f, qFrontVelo}}, P3);
        CMSISMat<3, 1> VP3 = {{vxc, vyc}};

        CMSISMat<3, 1> VP2_VP3 = VP2 - VP3;

        CMSISMat<3, 1> frontAngularVelo = VP2_VP3 / config.lowerLinkLength;

        qLowerFrontVelo = frontAngularVelo.data[2];

        // ||P4 - P3|| / l
        CMSISMat<3, 1> VP4 = tap::algorithms::cross({{0.0f, 0.0f, qBackVelo}}, P4);

        CMSISMat<3, 1> VP4_VP3 = VP4 - VP3;
        CMSISMat<3, 1> backAnglularVelo = VP4_VP3 / config.lowerLinkLength;

        qLowerBackVelo = backAnglularVelo.data[2];
    }

    void calculateLowerLegAngle()
    {
        // Calculate angle of the front lower leg
        const float deltaX_1 = (P2 - P3).data[0];
        const float deltaY_1 = (P2 - P3).data[1];
        qLowerFront = atan(deltaY_1 / deltaX_1);

        // Calculate angle of the back lower leg
        const float deltaX_2 = (P4 - P3).data[0];
        const float deltaY_2 = (P4 - P3).data[0];
        qLowerBack = atan(deltaX_2 / deltaY_2);
    }

    std::array<float, 2> calculateLegEnergy(BalstdLegConfig config);
};

class BalstdLeg
{
public:
    inline BalstdLeg(
        aruwsrc::control::motor::Tmotor_AK809& frontHipMotor,
        aruwsrc::control::motor::Tmotor_AK809& backHipMotor,
        tap::motor::MotorInterface& wheelMotor,
        const BalstdLegConfig config)
        : frontHipMotor(frontHipMotor),
          backHipMotor(backHipMotor),
          wheelMotor(wheelMotor),
          config(config)
    {
    }

    void initialize();

    void refresh();

    bool allMotorsOnline() const;

    void setThrust(const tap::algorithms::transforms::Vector thrust);

    void setWheelTorque(float torque);

    void updateState();

    inline BalstdLegState getState() const { return currState; }

    float updateCBF(BalstdLegState leg);

private:
    aruwsrc::control::motor::Tmotor_AK809& frontHipMotor;
    aruwsrc::control::motor::Tmotor_AK809& backHipMotor;
    tap::motor::MotorInterface& wheelMotor;

    const BalstdLegConfig config;

    BalstdLegState currState;

    inline float getFrontHipAngle() const
    {
        return tap::algorithms::WrappedFloat(
                   frontHipMotor.getEncoder()->getPosition().getWrappedValue(),
                   -M_PI_2,
                   3 * M_PI_2)
            .getWrappedValue();
    }

    inline float getBackHipAngle() const
    {
        return tap::algorithms::WrappedFloat(
                   backHipMotor.getEncoder()->getPosition().getWrappedValue(),
                   -M_PI_2,
                   3 * M_PI_2)
            .getWrappedValue();
    }

    void setHipTorques(float front, float back);

    static constexpr float M3508_TORQUE_CONSTANT =
        (tap::motor::DjiMotor::MAX_OUTPUT_C620 / 20.0f) / 0.21f;  // desOut/A / (Nm/A) = desOut/Nm

    float CBF_ENERGY_LIMIT;
    float maxTorque;
};

}  // namespace aruwsrc::balstd::chassis

#endif  // BALSTD_LEG_HPP_