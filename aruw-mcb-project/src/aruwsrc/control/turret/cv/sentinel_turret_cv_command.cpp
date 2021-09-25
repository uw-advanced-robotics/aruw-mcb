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

#include "sentinel_turret_cv_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/comprised_command.hpp"
#include "tap/drivers.hpp"

#include "../algorithms/turret_pid_control_algorithms.hpp"
#include "../turret_subsystem.hpp"
#include "aruwsrc/communication/serial/xavier_serial.hpp"
#include "aruwsrc/control/agitator/agitator_subsystem.hpp"

using namespace tap;
using namespace tap::algorithms;

namespace aruwsrc::control::turret
{
SentinelTurretCVCommand::SentinelTurretCVCommand(
    tap::Drivers *drivers,
    tap::control::turret::TurretSubsystemInterface *sentinelTurret,
    aruwsrc::agitator::AgitatorSubsystem *agitator,
    sentinel::firing::SentinelSwitcherSubsystem *switcher)
    : tap::control::ComprisedCommand(drivers),
      drivers(drivers),
      sentinelTurret(sentinelTurret),
      rotateAgitator(drivers, agitator, switcher),
      aimingAtTarget(false),
      lostTargetCounter(0),
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
    addSubsystemRequirement(agitator);
    addSubsystemRequirement(sentinelTurret);
    addSubsystemRequirement(switcher);
    comprisedCommandScheduler.registerSubsystem(agitator);
    comprisedCommandScheduler.registerSubsystem(sentinelTurret);
    comprisedCommandScheduler.registerSubsystem(switcher);
}

void SentinelTurretCVCommand::initialize()
{
    drivers->xavierSerial.beginAutoAim();
    pitchScanningUp = false;
    yawScanningRight = false;
    lostTargetCounter = 0;
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void SentinelTurretCVCommand::execute()
{
    if (drivers->xavierSerial.lastAimDataValid())
    {
        const auto &cvData = drivers->xavierSerial.getLastAimData();
        if (cvData.hasTarget)
        {
            aimingAtTarget = true;
            sentinelTurret->setYawSetpoint(cvData.yaw);
            sentinelTurret->setPitchSetpoint(cvData.pitch);

            if (fabs(sentinelTurret->getCurrentYawValue().difference(cvData.yaw)) <=
                    YAW_FIRE_ERROR_MARGIN &&
                fabs(sentinelTurret->getCurrentPitchValue().difference(cvData.pitch)) <=
                    PITCH_FIRE_ERROR_MARGIN)
            {
                if (!comprisedCommandScheduler.isCommandScheduled(&rotateAgitator))
                {
                    comprisedCommandScheduler.addCommand(&rotateAgitator);
                }
            }
        }
        else
        {
            scanForTarget();
        }
    }
    else
    {
        scanForTarget();
    }

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;
    runSinglePidYawChassisFrameController(dt, 0, yawPid, sentinelTurret);
    runSinglePidPitchChassisFrameController(dt, 0, pitchPid, sentinelTurret);

    comprisedCommandScheduler.run();
}

void SentinelTurretCVCommand::end(bool interrupted)
{
    drivers->xavierSerial.stopAutoAim();
    comprisedCommandScheduler.removeCommand(&rotateAgitator, interrupted);
}

void SentinelTurretCVCommand::updateScanningUp(
    const float motorSetpoint,
    const float minMotorSetpoint,
    const float maxMotorSetpoint,
    bool *axisScanningUp)
{
    if (motorSetpoint > maxMotorSetpoint)
    {
        *axisScanningUp = false;
    }
    else if (motorSetpoint < minMotorSetpoint)
    {
        *axisScanningUp = true;
    }
}

void SentinelTurretCVCommand::scanForTarget()
{
    // Increment aim counter and stop aiming at target if the target has been lost for some time
    if (aimingAtTarget)
    {
        lostTargetCounter++;

        if (lostTargetCounter > AIM_LOST_NUM_COUNTS)
        {
            lostTargetCounter = 0;
            aimingAtTarget = false;
        }
        else
        {
            return;
        }
    }

    const float pitchSetpoint = sentinelTurret->getPitchSetpoint();

    updateScanningUp(
        pitchSetpoint,
        TurretSubsystem::PITCH_MIN_ANGLE,
        TurretSubsystem::PITCH_MAX_ANGLE,
        &pitchScanningUp);

    sentinelTurret->setPitchSetpoint(
        pitchSetpoint + (pitchScanningUp ? SCAN_DELTA_ANGLE_PITCH : -SCAN_DELTA_ANGLE_PITCH));

    const float yawSetpoint = sentinelTurret->getYawSetpoint();

    updateScanningUp(
        yawSetpoint,
        TurretSubsystem::YAW_MIN_ANGLE,
        TurretSubsystem::YAW_MAX_ANGLE,
        &yawScanningRight);

    sentinelTurret->setYawSetpoint(
        yawSetpoint + (yawScanningRight ? SCAN_DELTA_ANGLE_YAW : -SCAN_DELTA_ANGLE_YAW));
}

}  // namespace aruwsrc::control::turret
