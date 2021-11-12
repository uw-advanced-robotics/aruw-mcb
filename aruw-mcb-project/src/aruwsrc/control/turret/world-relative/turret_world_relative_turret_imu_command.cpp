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

#include "turret_world_relative_turret_imu_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "../algorithms/turret_pid_turret_imu_world_rel.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

using namespace tap::sensors;
using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
TurretWorldRelativeTurretImuCommand::TurretWorldRelativeTurretImuCommand(
    tap::Drivers *drivers,
    TurretSubsystem *turretSubsystem)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      worldFrameYawSetpoint(TurretSubsystem::YAW_START_ANGLE, 0.0f, 360.0f),
      worldFramePitchSetpoint(TurretSubsystem::PITCH_START_ANGLE, 0.0f, 360.0f),
      yawPosPid(
          YAW_POS_P,
          YAW_POS_I,
          YAW_POS_D,
          YAW_POS_MAX_ERROR_SUM,
          YAW_POS_MAX_OUTPUT,
          YAW_POS_Q_DERIVATIVE_KALMAN,
          YAW_POS_R_DERIVATIVE_KALMAN,
          YAW_POS_Q_PROPORTIONAL_KALMAN,
          YAW_POS_R_PROPORTIONAL_KALMAN,
          YAW_POS_DEADZONE),
      yawVelPid(
          YAW_VEL_P,
          YAW_VEL_I,
          YAW_VEL_D,
          YAW_VEL_MAX_ERROR_SUM,
          YAW_VEL_MAX_OUTPUT,
          YAW_VEL_Q_DERIVATIVE_KALMAN,
          YAW_VEL_R_DERIVATIVE_KALMAN,
          YAW_VEL_Q_PROPORTIONAL_KALMAN,
          YAW_VEL_R_PROPORTIONAL_KALMAN,
          YAW_VEL_DEADZONE),
      pitchPosPid(
          PITCH_POS_P,
          PITCH_POS_I,
          PITCH_POS_D,
          PITCH_POS_MAX_ERROR_SUM,
          PITCH_POS_MAX_OUTPUT,
          PITCH_POS_Q_DERIVATIVE_KALMAN,
          PITCH_POS_R_DERIVATIVE_KALMAN,
          PITCH_POS_Q_PROPORTIONAL_KALMAN,
          PITCH_POS_R_PROPORTIONAL_KALMAN,
          PITCH_POS_DEADZONE),
      pitchVelPid(
          PITCH_VEL_P,
          PITCH_VEL_I,
          PITCH_VEL_D,
          PITCH_VEL_MAX_ERROR_SUM,
          PITCH_VEL_MAX_OUTPUT,
          PITCH_VEL_Q_DERIVATIVE_KALMAN,
          PITCH_VEL_R_DERIVATIVE_KALMAN,
          PITCH_VEL_Q_PROPORTIONAL_KALMAN,
          PITCH_VEL_R_PROPORTIONAL_KALMAN,
          PITCH_VEL_DEADZONE)
{
    addSubsystemRequirement(turretSubsystem);
}

bool TurretWorldRelativeTurretImuCommand::isReady() { return !isFinished(); }

bool TurretWorldRelativeTurretImuCommand::isFinished() const
{
    return !turretSubsystem->isOnline() || !drivers->turretMCBCanComm.isConnected();
}

void TurretWorldRelativeTurretImuCommand::initialize()
{
    // Reset PID controllers
    yawPosPid.reset();
    yawVelPid.reset();
    pitchPosPid.reset();
    pitchVelPid.reset();

    // Capture initial target angle in chassis frame and transform to world frame.
    worldFrameYawSetpoint.setValue(turret_imu_world_rel::transformChassisFrameToWorldFrame(
        turretSubsystem->getCurrentYawValue().getValue(),
        drivers->turretMCBCanComm.getYaw(),
        turretSubsystem->getYawSetpoint()));

    worldFramePitchSetpoint.setValue(turret_imu_world_rel::transformChassisFrameToWorldFrame(
        turretSubsystem->getCurrentPitchValue().getValue(),
        drivers->turretMCBCanComm.getPitch(),
        turretSubsystem->getPitchSetpoint()));

    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void TurretWorldRelativeTurretImuCommand::end(bool)
{
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
}

void TurretWorldRelativeTurretImuCommand::execute()
{
    // Update light to indicate IMU message received and turret controller running.
    imuMessageReceivedLEDBlinkCounter = (imuMessageReceivedLEDBlinkCounter + 1) % 100;
    drivers->leds.set(tap::gpio::Leds::Green, imuMessageReceivedLEDBlinkCounter > 50);

    turretSubsystem->updateCurrentTurretAngles();

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    const float pitchUserSetpoint =
        worldFramePitchSetpoint.getValue() +
        USER_PITCH_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretPitchInput();
    const float yawUserSetpoint =
        worldFrameYawSetpoint.getValue() +
        USER_YAW_INPUT_SCALAR * drivers->controlOperatorInterface.getTurretYawInput();

    turret_imu_world_rel::runDoublePidPitchWorldFrameController(
        dt,
        pitchUserSetpoint,
        drivers,
        TurretSubsystem::TURRET_CG_X,
        TurretSubsystem::TURRET_CG_Z,
        TurretSubsystem::GRAVITY_COMPENSATION_SCALAR,
        worldFramePitchSetpoint,
        pitchPosPid,
        pitchVelPid,
        turretSubsystem);

    turret_imu_world_rel::runDoublePidYawWorldFrameController(
        dt,
        yawUserSetpoint,
        drivers,
        worldFrameYawSetpoint,
        yawPosPid,
        yawVelPid,
        turretSubsystem);
}

}  // namespace aruwsrc::control::turret
