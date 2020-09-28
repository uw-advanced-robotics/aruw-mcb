/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "TurretChassisRelativePositionCommand.hpp"

#include <aruwlib/Drivers.hpp>
#include <aruwlib/algorithms/math_user_utils.hpp>

using namespace aruwlib::sensors;
using namespace aruwlib;

namespace aruwsrc
{
namespace turret
{
TurretChassisRelativePositionCommand::TurretChassisRelativePositionCommand(
    TurretSubsystem *subsystem,
    aruwlib::Drivers *drivers)
    : turretSubsystem(subsystem),
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
          PITCH_R_PROPORTIONAL_KALMAN),
      drivers(drivers)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem *>(subsystem));
}

void TurretChassisRelativePositionCommand::initialize()
{
    yawPid.reset();
    pitchPid.reset();
}

void TurretChassisRelativePositionCommand::execute()
{
    turretSubsystem->updateCurrentTurretAngles();
    runYawPositionController();
    runPitchPositionController();
}

void TurretChassisRelativePositionCommand::runYawPositionController()
{
    turretSubsystem->setYawTarget(
        turretSubsystem->getYawTarget() +
        USER_YAW_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretYawInput());
    float positionControllerError =
        turretSubsystem->getYawAngle().difference(turretSubsystem->getYawTarget());
    float pidOutput =
        yawPid.runController(positionControllerError, turretSubsystem->getYawVelocity());

    turretSubsystem->setYawMotorOutput(pidOutput);
}

void TurretChassisRelativePositionCommand::runPitchPositionController()
{
    // limit the yaw min and max angles
    turretSubsystem->setPitchTarget(
        turretSubsystem->getPitchTarget() +
        USER_PITCH_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretPitchInput());

    // position controller based on turret pitch gimbal and imu data
    float positionControllerError =
        turretSubsystem->getPitchAngle().difference(turretSubsystem->getPitchTarget());

    float pidOutput =
        pitchPid.runController(positionControllerError, turretSubsystem->getPitchVelocity());

    // gravity compensation
    pidOutput +=
        PITCH_GRAVITY_COMPENSATION_KP *
        cosf(aruwlib::algorithms::degreesToRadians(turretSubsystem->getPitchAngleFromCenter()));

    turretSubsystem->setPitchMotorOutput(pidOutput);
}

void TurretChassisRelativePositionCommand::end(bool) {}
}  // namespace turret
}  // namespace aruwsrc
