#include "aruwsrc/robot/dart/dart_reloader_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "dart_constants.hpp"
namespace aruwsrc::robot::dart
{
DartReloaderSubsystem::DartReloaderSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motor)
    : Subsystem(drivers),
      motor(motor),
      pidController(DART_RELOADER_PID_CONFIG)
{
}
void DartReloaderSubsystem::initialize()
{
    motor.initialize();
    pidController.reset();
}
void DartReloaderSubsystem::setSetpoint(float32_t setpoint) { this->setpoint = setpoint; }

bool DartReloaderSubsystem::atSetpoint()
{
    float error = motor.getEncoder()->getPosition().getUnwrappedValue() - setpoint;
    return tap::algorithms::compareFloatClose(error, 0.0f, DART_RELOADER_PID_CONFIG.errDeadzone);
}
void DartReloaderSubsystem::refresh()
{
    float currentPosition = motor.getEncoder()->getPosition().getUnwrappedValue();
    float error = setpoint - currentPosition;
    float errorDerivative = -motor.getEncoder()->getVelocity();
    float output = pidController.runController(
        error,
        errorDerivative,
        0.002f);
    motor.setDesiredOutput(output);
    position = currentPosition;
}

void DartReloaderSubsystem::refreshSafeDisconnect() { motor.setDesiredOutput(0); }

}  // namespace aruwsrc::robot::dart