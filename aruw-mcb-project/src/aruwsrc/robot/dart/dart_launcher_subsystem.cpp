#include "dart_launcher_subsystem.hpp"
#include "aruwsrc/control/turret/constants/turret_constants.hpp"

#include "dart_drivers.hpp"
#include "dart_turret_constants.hpp"
#include "dart_constants.hpp"
using namespace aruwsrc::control::turret;

namespace aruwsrc::robot::dart
{
DartLauncherSubsystem::DartLauncherSubsystem(
    tap::Drivers *drivers,
    tap::motor::MotorInterface& pullMotor)
    : Subsystem(drivers),
      motor(pullMotor){};

  void DartLauncherSubsystem::initialize() {
    motor.initialize();
    lastTime = tap::arch::clock::getTimeMilliseconds();
  }

  void DartLauncherSubsystem::moveMotor(int32_t power) {
    motor.setDesiredOutput(power);
  }

  void DartLauncherSubsystem::setSetpoint(float newSetpoint) {
    setpoint = newSetpoint;
  }

  bool DartLauncherSubsystem::isBeamBroken() {
    return drivers->digital.read(BEAMBREAK_PORT);
  }

  // void DartLauncherSubsystem::setServoOpen() {
  //   servo.setTargetPwm(servo.getMaxPWM());
  // }

  // void DartLauncherSubsystem::setServoClosed() {
  //   servo.setTargetPwm(servo.getMinPWM());
  // }

  void DartLauncherSubsystem::refresh() {
    // float error = setpoint - motor.getPositionUnwrapped();
    // float errorDerivative = motor.getShaftRPM() / 1000 / 60; //rotations per millisecond
    // float timeDifference = tap::arch::clock::getTimeMilliseconds() - lastTime; //time in milliseconds
    // lastTime = tap::arch::clock::getTimeMilliseconds();
    // pid.runController(error, errorDerivative, timeDifference); 
    // motor.setDesiredOutput(pid.getOutput());

    // servo.updateSendPwmRamp();

  }

  void DartLauncherSubsystem::refreshSafeDisconnect() {
    motor.setDesiredOutput(0);
  }
}