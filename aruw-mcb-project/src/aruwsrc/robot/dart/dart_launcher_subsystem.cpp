#include "dart_launcher_subsystem.hpp"

#include "dart_drivers.hpp"

namespace dart::subsystem
{
DartLauncherSubsystem::DartLauncherSubsystem(
    tap::Drivers& drivers,
    tap::motor::MotorInterface& pullMotor)
    : Subsystem(&drivers),
      motor(pullMotor) {};

}