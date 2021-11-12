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

#include "turret_world_relative_chassis_imu_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

#include "../algorithms/chassis_frame_turret_controller.hpp"
#include "../algorithms/world_frame_chassis_imu_turret_controller.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::sensors;

namespace aruwsrc::control::turret
{
TurretWorldRelativeChassisImuCommand::TurretWorldRelativeChassisImuCommand(
    aruwsrc::Drivers *drivers,
    TurretSubsystem *turretSubsystem)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      worldFrameYawSetpoint(TurretSubsystem::YAW_START_ANGLE, 0.0f, 360.0f),
      chassisFrameInitImuYawAngle(0.0f),
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

void TurretWorldRelativeChassisImuCommand::initialize()
{
    yawPid.reset();
    pitchPid.reset();

    chassisFrameInitImuYawAngle = drivers->mpu6500.getYaw();

    worldFrameYawSetpoint.setValue(turretSubsystem->getYawSetpoint());

    prevTime = tap::arch::clock::getTimeMilliseconds();
}

bool TurretWorldRelativeChassisImuCommand::isReady() { return !isFinished(); }

void TurretWorldRelativeChassisImuCommand::execute()
{
    turretSubsystem->updateCurrentTurretAngles();

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    const float pitchUserInput =
        turretSubsystem->getPitchSetpoint() +
        USER_PITCH_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretPitchInput();
    const float yawUserInput =
        worldFrameYawSetpoint.getValue() +
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
    WorldFrameChassisImuTurretController::runYawPidController(
        *drivers,
        dt,
        yawUserInput,
        chassisFrameInitImuYawAngle,
        &worldFrameYawSetpoint,
        &yawPid,
        turretSubsystem);
}

bool TurretWorldRelativeChassisImuCommand::isFinished() const
{
    return !turretSubsystem->isOnline() || !drivers->mpu6500.isRunning();
}

void TurretWorldRelativeChassisImuCommand::end(bool)
{
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
}

}  // namespace aruwsrc::control::turret
