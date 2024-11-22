#include "dart_launcher_subsystem.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"

#include "dart_drivers.hpp"
#include "dart_turret_constants.hpp"
using namespace aruwsrc::control::turret;

namespace dart::subsystem
{
DartLauncherSubsystem::DartLauncherSubsystem(
    tap::Drivers& drivers,
    tap::motor::MotorInterface& pullMotor, tap::motor::Servo& servo)
    : Subsystem(&drivers),
      motor(pullMotor), servo(servo){};

  void DartLauncherSubsystem::initialize() {
    motor.initialize();
    lastTime = tap::arch::clock::getTimeMilliseconds();
  }

  void DartLauncherSubsystem::setSetpoint(float newSetpoint) {
    setpoint = newSetpoint;
  }

  void DartLauncherSubsystem::setServoOpen() {
    servo.setTargetPwm(servo.getMaxPWM());
  }

  void DartLauncherSubsystem::setServoClosed() {
    servo.setTargetPwm(servo.getMinPWM());
  }

  void DartLauncherSubsystem::refresh() {
    float error = setpoint - motor.getPositionUnwrapped();
    float errorDerivative = motor.getShaftRPM() / 1000 / 60; //rotations per millisecond
    float timeDifference = tap::arch::clock::getTimeMilliseconds() - lastTime; //time in milliseconds
    lastTime = tap::arch::clock::getTimeMilliseconds();
    pid.runController(error, errorDerivative, timeDifference); 
    motor.setDesiredOutput(pid.getOutput());

    servo.updateSendPwmRamp();
  }
}