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

#include "../algorithms/chassis_frame_turret_controller.hpp"
#include "../turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::turret
{
TurretChassisRelativeCommand::TurretChassisRelativeCommand(
    aruwsrc::Drivers *drivers,
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

void TurretChassisRelativeCommand::initialize()
{
    pitchPid.reset();
    yawPid.reset();
    prevTime = tap::arch::clock::getTimeMilliseconds();
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

    // updates the turret pitch setpoint based on the user input, runs the PID controller, and sets
    // the turret subsystem's desired pitch output
    ChassisFrameTurretController::runPitchPidController(
        dt,
        pitchUserInput,
        TurretSubsystem::TURRET_CG_X,
        TurretSubsystem::TURRET_CG_Z,
        TurretSubsystem::GRAVITY_COMPENSATION_SCALAR,
        &pitchPid,
        turretSubsystem);

    // updates the turret yaw setpoint based on the user input, runs the PID controller, and sets
    // the turret subsystem's desired yaw output
    ChassisFrameTurretController::runYawPidController(dt, yawUserInput, &yawPid, turretSubsystem);
}

bool TurretChassisRelativeCommand::isFinished() const { return !turretSubsystem->isOnline(); }

void TurretChassisRelativeCommand::end(bool)
{
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
}

}  // namespace aruwsrc::control::turret
