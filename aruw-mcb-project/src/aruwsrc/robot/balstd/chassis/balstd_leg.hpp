#ifndef BALSTD_LEG_HPP_
#define BALSTD_LEG_HPP_

#include "tap/algorithms/transforms/vector.hpp"
#include "tap/motor/motor_interface.hpp"

#include "aruwsrc/control/motor/tmotor_ak80_9.hpp"
#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::control::balstd
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
};
struct BalstdLegState
{
    float qFront, qBack;          // angles of upper linkages in radians
    float qFrontVelo, qBackVelo;  // velocities of upper linkages in radians/s

    float xc, yc;                    // coordinates of wheel axle wrt hip center
    float kneesWidthX, kneesWidthY;  // components of distance between knees

    float L, theta;  // pendulum length and angle wrt hip center

    BalstdLegState()
        : qFront(0),
          qBack(0),
          qFrontVelo(0),
          qBackVelo(0),
          xc(0),
          yc(0),
          kneesWidthX(0),
          kneesWidthY(0),
          L(0),
          theta(0)
    {
    }

    void calculateForwardKinematics(BalstdLegConfig config)
    {
        // knee coordinates
        CMSISMat<2, 1> P2 = CMSISMat<2, 1>(
            {config.upperLinkLength * cos(qFront), config.upperLinkLength * sin(qFront)});

        CMSISMat<2, 1> P4 = CMSISMat<2, 1>(
            {config.upperLinkLength * cos(qBack) - config.fixedLinkLength,
             config.upperLinkLength * sin(qBack)});

        // wheel coordinates
        // ||P2-Ph|| = (a2^2 - a3^2 + ||P4-P2||^2) / (2*||P4-P2||)
        // a2 and a3 are the upper leg links and are the same
        // P4-P2
        CMSISMat<2, 1> P4_P2 = P4 - P2;

        kneesWidthX = P4_P2.data[0];
        kneesWidthY = P4_P2.data[1];

        // ||P4-P2||
        float P4_P2_mag = sqrtf(P4_P2.data[0] * P4_P2.data[0] + P4_P2.data[1] * P4_P2.data[1]);

        // ||P2-Ph||
        float P2_Ph_mag = P4_P2_mag / 2;

        // Ph = P2 + ||P2-Ph|| / ||P2-P4|| * (P4-P2)
        CMSISMat<2, 1> Ph = P2 + P2_Ph_mag / P4_P2_mag * P4_P2;

        // ||P3-Ph|| = sqrt(a2^2 - ||P2-Ph||^2)
        float P3_Ph_mag =
            sqrtf(config.lowerLinkLength * config.lowerLinkLength - P2_Ph_mag * P2_Ph_mag);

        // P3 = Ph ± ||P3-Ph|| / ||P2-P4|| * (P4-P2)
        P4_P2.data[1] *= -1;
        CMSISMat<2, 1> P3 = Ph + P3_Ph_mag / P4_P2_mag * P4_P2;

        xc = P3.data[0];
        yc = P3.data[1];
    }

    void calculatePendulumState()
    {
        L = atan2(xc, yc);
        theta = sqrt(xc * xc + yc * yc);
    }
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

    bool allMotorsOnline() const;

    void setThrust(const tap::algorithms::transforms::Vector thrust);

    void setWheelTorque(float torque);

    void updateState();

    inline BalstdLegState getState() const { return currState; }

private:
    aruwsrc::control::motor::Tmotor_AK809& frontHipMotor;
    aruwsrc::control::motor::Tmotor_AK809& backHipMotor;
    tap::motor::MotorInterface& wheelMotor;

    const BalstdLegConfig config;

    BalstdLegState currState;

    tap::algorithms::CMSISMat<2, 2> jacobianTranspose;

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
    void setBackHipMotorTorque(float torque);

    void calculateJacobianTranspose();
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_LEG_HPP_