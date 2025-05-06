#include "balstd_leg.hpp"

using aruwsrc::control::motor::Tmotor_AK809;
using tap::algorithms::CMSISMat;

namespace aruwsrc::control::balstd
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
    CMSISMat<2, 1> torques = jacobianTranspose * CMSISMat<2, 1>({thrust.x(), thrust.y()});

    setHipTorques(torques.data[0], torques.data[1]);
}

void BalstdLeg::setWheelTorque(float torque)
{
    wheelMotor.setDesiredOutput(torque * M3508_TORQUE_CONSTANT);
}

void BalstdLeg::setHipTorques(float front, float back)
{
    // soft stops
    if (currState.qFront <= config.frontHipOuterLimit && front < 0) front = 0;
    if (currState.qFront >= config.frontHipInnerLimit && front > 0) front = 0;

    if (currState.qBack <= config.backHipInnerLimit && back > 0) back = 0;
    if (currState.qBack >= config.backHipOuterLimit && back < 0) back = 0;

    frontHipMotor.setDesiredOutput(front * Tmotor_AK809::TORQUE_CONSTANT);
    backHipMotor.setDesiredOutput(back * Tmotor_AK809::TORQUE_CONSTANT);
}

void BalstdLeg::updateState()
{
    currState.qFront = getFrontHipAngle();
    currState.qBack = getBackHipAngle();
    currState.wheelVel = wheelMotor.getEncoder()->getVelocity();
    currState.calculateForwardKinematics(config);

    currState.qFrontVelo = frontHipMotor.getEncoder()->getVelocity();
    currState.qBackVelo = backHipMotor.getEncoder()->getVelocity();

    calculateJacobianTranspose();
}

void BalstdLeg::calculateJacobianTranspose()
{
    float p1x2 = -config.upperLinkLength * sin(currState.qFront);
    float p1y2 = config.upperLinkLength * cos(currState.qFront);
    float p5x4 = -config.upperLinkLength * sin(currState.qBack);
    float p5y4 = config.upperLinkLength * cos(currState.qBack);

    float d = sqrt(
        currState.kneesWidthX * currState.kneesWidthX +
        currState.kneesWidthY * currState.kneesWidthY);
    float h = sqrt(config.lowerLinkLength * config.lowerLinkLength - d * d / 4);

    float p1d = -(currState.kneesWidthX * p1x2 + currState.kneesWidthY * p1y2) / d;
    float p5d = (currState.kneesWidthX * p5x4 + currState.kneesWidthY * p5y4) / d;
    float p1h = -d * p1d / h / 4;
    float p5h = -d * p5d / h / 4;

    float p1x3 = p1x2 / 2 - h / d * p1y2 + (p1h * d - p1d * h) / (d * d) * currState.kneesWidthY;
    float p1y3 = p1y2 / 2 + h / d * p1x2 - (p1h * d - p1d * h) / (d * d) * currState.kneesWidthX;
    float p5x3 = p5x4 / 2 + h / d * p5y4 + (p5h * d - p5d * h) / (d * d) * currState.kneesWidthY;
    float p5y3 = p5y4 / 2 - h / d * p5x4 - (p5h * d - p5d * h) / (d * d) * currState.kneesWidthX;

    jacobianTranspose = CMSISMat<2, 2>({p1x3, p1y3, p5x3, p5y3});
}


}  // namespace aruwsrc::control::balstd