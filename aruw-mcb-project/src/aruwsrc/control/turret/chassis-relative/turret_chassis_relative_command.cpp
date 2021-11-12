/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "turret_chassis_relative_command.hpp"

#include "tap/drivers.hpp"

#include "../algorithms/turret_pid_chassis_rel.hpp"
#include "../turret_subsystem.hpp"

namespace aruwsrc::control::turret
{
TurretChassisRelativeCommand::TurretChassisRelativeCommand(
    tap::Drivers *drivers,
    tap::control::turret::TurretSubsystemInterface *turretSubsystem)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
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
    addSubsystemRequirement(turretSubsystem);
}

bool TurretChassisRelativeCommand::isReady() { return turretSubsystem->isOnline(); }

bool TurretChassisRelativeCommand::isFinished() const { return !turretSubsystem->isOnline(); }

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

    const float pitchUserInput =
        turretSubsystem->getPitchSetpoint() +
        USER_PITCH_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretPitchInput();
    const float yawUserInput =
        turretSubsystem->getYawSetpoint() +
        USER_YAW_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretYawInput();

    chassis_rel::runSinglePidPitchChassisFrameController(
        dt,
        pitchUserInput,
        TurretSubsystem::TURRET_CG_X,
        TurretSubsystem::TURRET_CG_Z,
        TurretSubsystem::GRAVITY_COMPENSATION_SCALAR,
        pitchPid,
        turretSubsystem);

    chassis_rel::runSinglePidYawChassisFrameController(dt, yawUserInput, yawPid, turretSubsystem);
}
}  // namespace aruwsrc::control::turret
