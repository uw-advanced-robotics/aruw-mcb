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
    float xc, yc;         // coordinates of wheel axle wrt hip center
    float L, theta;       // pendulum length and angle wrt hip center
};

struct BalstdLegConfig
{
    float upperLinkLength;  // meters
    float lowerLinkLength;  // meters
    float fixedLinkLength;  // meters

    float frontHipOuterLimit = modm::toRadian(-15);  // radians
    float frontHipInnerLimit = modm::toRadian(90);   // radians
    float backHipOuterLimit = modm::toRadian(195);   // radians
    float backHipInnerLimit = modm::toRadian(90);    // radians
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
    tap::motor::MotorInterface& frontHipMotor;  // 0
    tap::motor::MotorInterface& backHipMotor;   // 1
    tap::motor::MotorInterface& wheelMotor;

    const BalstdLegConfig config;

    BalstdLegState currState;

    tap::algorithms::CMSISMat<2, 2> jacobianTranspose;

    void setFrontHipMotorTorque(float torque);
    void setBackHipMotorTorque(float torque);

    void calculateJacobianTranspose();
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_LEG_HPP_