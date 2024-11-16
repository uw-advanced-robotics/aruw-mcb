#include "dart_launcher_subsystem.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"

#include "dart_drivers.hpp"
#include "dart_turret_constants.hpp"
using namespace aruwsrc::control::turret;

namespace dart::subsystem
{
DartLauncherSubsystem::DartLauncherSubsystem(
    tap::Drivers& drivers,
    tap::motor::MotorInterface& pullMotor)
    : Subsystem(&drivers),
      motor(pullMotor){};

  void DartLauncherSubsystem::initialize() {
    motor.initialize();
    lastTime = tap::arch::clock::getTimeMilliseconds();
  }

  void DartLauncherSubsystem::setSetpoint(float newSetpoint) {
    setpoint = newSetpoint;
  }

  void DartLauncherSubsystem::refresh() {
    float error = setpoint - motor.getPositionUnwrapped() - zeroOffset; //how do we convert encoder ticks
    float errorDerivative = motor.getShaftRPM() / 1000 / 60; //rotations per millisecond
    float timeDifference = tap::arch::clock::getTimeMilliseconds() - lastTime; //time in milliseconds
    lastTime = tap::arch::clock::getTimeMilliseconds();
    pid.runController(error, errorDerivative, timeDifference); 
    motor.setDesiredOutput(pid.getOutput());
  }
}