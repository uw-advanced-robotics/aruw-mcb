#include "dart_yaw_subsystem.hpp" 
#include "dart_drivers.hpp"
#include "dart_constants.hpp"

namespace dart::dart_yaw_subsystem 
{
DartYawSubsystem::DartYawSubsystem(
    tap::Drivers& drivers, tap::motor::MotorInterface& deadMotor, tap::motor::MotorInterface& yawMotor) 
    : Subsystem(&drivers), deadMotor(deadMotor), yawMotor(yawMotor), digital(drivers.digital)
    {};
void DartYawSubsystem::initialize() {
    deadMotor.initialize();
    yawMotor.initialize();
    lastTime = tap::arch::clock::getTimeMilliseconds();
}

void DartYawSubsystem::setSetpoint(float newSetpoint) {
    setpoint = newSetpoint;
}

bool DartYawSubsystem::getDigitalPin() {
    digital.read(aruwsrc::control::turret::limitSwitchPin);
}

void DartYawSubsystem::reset() {
    deadMotor.resetEncoderValue();
}

void DartYawSubsystem::refresh() {
    float error = setpoint - deadMotor.getPositionUnwrapped();
    float errorDerivative = deadMotor.getShaftRPM() / 1000 / 60;
    float timeDifference = tap::arch::clock::getTimeMilliseconds() - lastTime;
    lastTime = tap::arch::clock::getTimeMilliseconds();
    pid.runController(error, errorDerivative, timeDifference);
    yawMotor.setDesiredOutput(pid.getOutput());

}

}