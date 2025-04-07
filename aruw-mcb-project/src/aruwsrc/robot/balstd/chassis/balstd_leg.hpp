#ifndef BALSTD_LEG_HPP_
#define BALSTD_LEG_HPP_

#include "tap/algorithms/transforms/vector.hpp"
#include "tap/motor/motor_interface.hpp"

#include "modm/math/geometry/angle.hpp"

namespace aruwsrc::control::balstd
{
using tap::algorithms::transforms::Vector;
struct BalstdLegState
{
    float qFront, qBack;  // angles of upper linkages in radians

    float xc, yc;                    // coordinates of wheel axle wrt hip center
    float kneesWidthX, kneesWidthY;  // components of distance between knees

    float L, theta;  // pendulum length and angle wrt hip center

    void calculateForwardKinematics(BalstdLegConfig config)
    {
        // knee coordinates
        float x2 = config.upperLinkLength * cos(qFront);
        float y2 = config.upperLinkLength * sin(qFront);
        float x4 = config.upperLinkLength * cos(qBack) - config.fixedLinkLength;
        float y4 = config.upperLinkLength * sin(qBack);

        kneesWidthX = x4 - x2;
        kneesWidthY = y4 - y2;

        // wheel coordinates (TODO)
        xc = 0;
        yc = 0;
    }

    void calculatePendulumState()
    {
        L = atan2(xc, yc);
        theta = sqrt(xc * xc + yc * yc);
    }
};

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

class BalstdLeg
{
public:
    inline BalstdLeg(
        tap::motor::MotorInterface& frontHipMotor,
        tap::motor::MotorInterface& backHipMotor,
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
    tap::motor::MotorInterface& frontHipMotor;
    tap::motor::MotorInterface& backHipMotor;
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