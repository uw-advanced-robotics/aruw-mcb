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

#include "turret_cv_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "../algorithms/turret_pid_control_algorithms.hpp"

using namespace tap::arch::clock;

namespace aruwsrc::control::turret
{
TurretCVCommand::TurretCVCommand(tap::Drivers *drivers, TurretSubsystem *subsystem)
    : drivers(drivers),
      turretSubsystem(subsystem),
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
    addSubsystemRequirement(subsystem);
}

void TurretCVCommand::initialize()
{
    drivers->xavierSerial.beginAutoAim();
    yawPid.reset();
    pitchPid.reset();
}

void TurretCVCommand::execute()
{
    if (drivers->xavierSerial.lastAimDataValid())
    {
        const auto &cvData = drivers->xavierSerial.getLastAimData();
        if (cvData.hasTarget)
        {
            turretSubsystem->setYawSetpoint(cvData.yaw);
            turretSubsystem->setPitchSetpoint(cvData.pitch);
        }
    }

    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    runSinglePidYawChassisFrameController(dt, 0.0f, yawPid, turretSubsystem);
    runSinglePidPitchChassisFrameController(dt, 0.0f, pitchPid, turretSubsystem);
}

void TurretCVCommand::end(bool)
{
    drivers->xavierSerial.stopAutoAim();
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
}

}  // namespace aruwsrc::control::turret
