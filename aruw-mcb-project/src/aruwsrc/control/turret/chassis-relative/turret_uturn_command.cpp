#include "turret_uturn_command.hpp"
#include "tap/drivers.hpp"

#include "turret_chassis_relative_command.hpp"

namespace aruwsrc::control::turret
{
TurretUTurnCommand::TurretUTurnCommand(
    tap::control::turret::TurretSubsystemInterface *turretSubsystem,
    const float targetOffsetToTurn)
    : turretSubsystem(turretSubsystem),
      targetOffsetToTurn(targetOffsetToTurn)
{
}

}  // namespace aruwsrc::control::turret
