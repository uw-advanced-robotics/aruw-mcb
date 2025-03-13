#include "balstd_leg.hpp"

using tap::algorithms::CMSISMat;

namespace aruwsrc::control::balstd
{

void BalstdLeg::initialize()
{
    frontHipMotor.initialize();
    backHipMotor.initialize();
    wheelMotor.initialize();
}

bool BalstdLeg::allMotorsOnline() const
{
    return frontHipMotor.isMotorOnline() && backHipMotor.isMotorOnline() &&
           wheelMotor.isMotorOnline();
}

void BalstdLeg::setThrust(const Vector thrust)
{
    CMSISMat<2, 1> torques = jacobianTranspose * CMSISMat<2, 1>({thrust.x(), thrust.y()});

    setFrontHipMotorTorque(torques.data[0]);
    setBackHipMotorTorque(torques.data[1]);
}

void BalstdLeg::setWheelTorque(float torque)
{
    // TODO: once characterized
    // wheelMotor.setDesiredOutput(torque);
}

void BalstdLeg::setFrontHipMotorTorque(float torque)
{
    // TODO: wait for encoders mr for radian position getter, this is wrong rn
    if (frontHipMotor.getEncoderWrapped() <= FRONT_HIP_OUTER_LIMIT && torque < 0) torque = 0;
    if (frontHipMotor.getEncoderWrapped() >= FRONT_HIP_INNER_LIMIT && torque > 0) torque = 0;

    // TODO: once characterized
    // frontHipMotor.setDesiredOutput(torque);
}

void BalstdLeg::setBackHipMotorTorque(float torque)
{
    // TODO: wait for encoders mr for radian position getter, this is wrong rn
    if (backHipMotor.getEncoderWrapped() <= BACK_HIP_INNER_LIMIT && torque < 0) torque = 0;
    if (backHipMotor.getEncoderWrapped() >= BACK_HIP_OUTER_LIMIT && torque > 0) torque = 0;

    // TODO: once characterized
    // backHipMotor.setDesiredOutput(torque);
}

void BalstdLeg::updateState()
{
    // TODO: wait for motor/encoder tap mr lol
    currState.qFront = frontHipMotor.getEncoderWrapped();
    currState.qBack = backHipMotor.getEncoderWrapped();

    calculateJacobianTranspose();
}

void BalstdLeg::calculateJacobianTranspose()
{
    // could be calculated with fk and stored in state
    float x2 = config.upperLinkLength * cos(currState.qFront);
    float y2 = config.upperLinkLength * sin(currState.qFront);
    float x4 = config.upperLinkLength * cos(currState.qBack) - config.fixedLinkLength;
    float y4 = config.upperLinkLength * sin(currState.qBack);

    float p1x2 = -config.upperLinkLength * sin(currState.qFront);
    float p1y2 = config.upperLinkLength * cos(currState.qFront);
    float p5x4 = -config.upperLinkLength * sin(currState.qBack);
    float p5y4 = config.upperLinkLength * cos(currState.qBack);

    float d = sqrt((x4 - x2) * (x4 - x2) + (y4 - y2) * (y4 - y2));
    float h = sqrt(config.lowerLinkLength * config.lowerLinkLength - d * d / 4);

    float p1d = ((x4 - x2) * (-p1x2) + (y4 - y2) * (-p1y2)) / d;
    float p5d = ((x4 - x2) * p5x4 + (y4 - y2) * p5y4) / d;
    float p1h = -d * p1d / h / 4;
    float p5h = -d * p5d / h / 4;

    float p1x3 = p1x2 / 2 - h / d * p1y2 + (p1h * d - p1d * h) / (d * d) * (y4 - y2);
    float p1y3 = p1y2 / 2 + h / d * p1x2 - (p1h * d - p1d * h) / (d * d) * (x4 - x2);
    float p5x3 = p5x4 / 2 + h / d * p5y4 + (p5h * d - p5d * h) / (d * d) * (y4 - y2);
    float p5y3 = p5y4 / 2 - h / d * p5x4 - (p5h * d - p5d * h) / (d * d) * (x4 - x2);

    jacobianTranspose = CMSISMat<2, 2>({p1x3, p1y3, p5x3, p5y3});
}

}  // namespace aruwsrc::control::balstd