#ifndef BALSTD_LEG_HPP_
#define BALSTD_LEG_HPP_

#include "tap/algorithms/transforms/vector.hpp"
#include "tap/motor/motor_interface.hpp"

using tap::algorithms::transforms::Vector;

namespace aruwsrc::control::balstd
{
typedef struct BalstdLegState
{
    float qFront, qBack;  // angles of upper linkages in radians
    float xc, yc;         // coordinates of wheel axle wrt hip center
    float L, theta;       // pendulum length and angle wrt hip center
};

typedef struct BalstdLegConfig
{
    float upperLinkLength;  // meters
    float lowerLinkLength;  // meters
    float fixedLinkLength;  // meters
};

class BalstdLeg
{
public:
    inline BalstdLeg(
        tap::motor::MotorInterface& frontHipMotor,
        tap::motor::MotorInterface& backHipMotor,
        tap::motor::MotorInterface& wheelMotor,
        BalstdLegConfig& config)
        : frontHipMotor(frontHipMotor),
          backHipMotor(backHipMotor),
          wheelMotor(wheelMotor),
          config(config)
    {
    }

    void setThrust(tap::algorithms::transforms::Vector& thrust);

    void setWheelTorque(float torque);

    void updateState();

private:
    tap::motor::MotorInterface& frontHipMotor;
    tap::motor::MotorInterface& backHipMotor;
    tap::motor::MotorInterface& wheelMotor;

    BalstdLegConfig config;

    BalstdLegState currState;

    tap::algorithms::CMSISMat<2, 2> jacobianTranspose;

    void setHipMotorTorque(tap::motor::MotorInterface& hipMotor, float torque);

    void calculateJacobianTranspose();
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_LEG_HPP_