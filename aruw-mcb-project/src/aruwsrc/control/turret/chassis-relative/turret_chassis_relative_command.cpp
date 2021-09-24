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
      pitchInputScalar(pitchInputScalar),
      yawPid(
          YAW_P,
          YAW_I,
          YAW_D,
          YAW_MAX_ERROR_SUM,
          YAW_MAX_OUTPUT,
          YAW_Q_DERIVATIVE_KALMAN,
          YAW_R_DERIVATIVE_KALMAN,
          YAW_Q_PROPORTIONAL_KALMAN,
          YAW_R_PROPORTIONAL_KALMAN),
      pitchPid(
          PITCH_P,
          PITCH_I,
          PITCH_D,
          PITCH_MAX_ERROR_SUM,
          PITCH_MAX_OUTPUT,
          PITCH_Q_DERIVATIVE_KALMAN,
          PITCH_R_DERIVATIVE_KALMAN,
          PITCH_Q_PROPORTIONAL_KALMAN,
          PITCH_R_PROPORTIONAL_KALMAN)
{
}

void TurretChassisRelativeCommand::initialize()
{
    pitchPid.reset();
    yawPid.reset();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void TurretChassisRelativeCommand::end(bool)
{
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
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
