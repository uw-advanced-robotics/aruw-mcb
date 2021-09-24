#include "turret_chassis_relative_command.hpp"

#include "tap/drivers.hpp"

#include "../algorithms/turret_pid_control_algorithms.hpp"

namespace aruwsrc::control::turret
{
TurretChassisRelativeCommand::TurretChassisRelativeCommand(
    tap::Drivers *drivers,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem,
    const float yawInputScalar,
    const float pitchInputScalar)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawInputScalar(yawInputScalar),
      pitchInputScalar(pitchInputScalar)
{
}

void TurretChassisRelativeCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    runSinglePidPitchChassisFrameController(
        dt,
        pitchInputScalar * drivers->controlOperatorInterface.getTurretPitchInput(),
        pitchPid,
        turretSubsystem);

    runSinglePidYawChassisFrameController(
        dt,
        yawInputScalar * drivers->controlOperatorInterface.getTurretYawInput(),
        yawPid,
        turretSubsystem);
}
}  // namespace aruwsrc::control::turret
