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

#include "../algorithms/turret_controller_interface.hpp"
#include "../turret_subsystem.hpp"
#include "aruwsrc/communication/serial/legacy_vision_coprocessor.hpp"
#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::algorithms;

namespace aruwsrc::control::turret::cv
{
SentinelTurretCVCommand::SentinelTurretCVCommand(
    aruwsrc::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    aruwsrc::agitator::AgitatorSubsystem *agitator,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController)
    : tap::control::ComprisedCommand(drivers),
      drivers(drivers),
      turretSubsystem(turretSubsystem),
      rotateAgitator(
          drivers,
          agitator,
          AGITATOR_ROTATE_ANGLE,
          AGITATOR_MAX_UNJAM_ANGLE,
          AGITATOR_ROTATE_TIME,
          true,
          10),
      aimingAtTarget(false),
      lostTargetCounter(0),
      yawController(yawController),
      pitchController(pitchController)
{
    addSubsystemRequirement(agitator);
    addSubsystemRequirement(turretSubsystem);
    comprisedCommandScheduler.registerSubsystem(agitator);
    comprisedCommandScheduler.registerSubsystem(turretSubsystem);
}

bool SentinelTurretCVCommand::isReady() { return !isFinished(); }

void SentinelTurretCVCommand::initialize()
{
    drivers->legacyVisionCoprocessor.beginAutoAim();
    pitchScanningUp = false;
    yawScanningRight = false;
    lostTargetCounter = 0;
    prevTime = tap::arch::clock::getTimeMilliseconds();
    yawController->initialize();
    pitchController->initialize();
}

void SentinelTurretCVCommand::execute()
{
    // check validity of aim data
    if (drivers->legacyVisionCoprocessor.lastAimDataValid())
    {
        // aim data valid, get aim data
        const auto &cvData = drivers->legacyVisionCoprocessor.getLastAimData();
        if (cvData.hasTarget)
        {
            // a target has been acquired, set target setpoints of turret
            // in the old system, we could not have a target and still have valid aim data
            aimingAtTarget = true;
            turretSubsystem->setYawSetpoint(cvData.yaw);
            turretSubsystem->setPitchSetpoint(cvData.pitch);

            // we have a target and we are close to the target setpoint (the turret is pointing at
            // the target), so rotate agitator to launch a projectile if not already in process of
            // launching
            if (fabs(turretSubsystem->getCurrentYawValue().difference(cvData.yaw)) <=
                    YAW_FIRE_ERROR_MARGIN &&
                fabs(turretSubsystem->getCurrentPitchValue().difference(cvData.pitch)) <=
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
            // no target acquired, scan
            scanForTarget();
        }
    }
    else
    {
        // no valid aim data, scan
        scanForTarget();
    }

    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    // updates the turret pitch setpoint based on CV input, runs the PID controller, and sets
    // the turret subsystem's desired pitch output
    pitchController->runController(dt, turretSubsystem->getPitchSetpoint());

    // updates the turret yaw setpoint based on CV input, runs the PID controller, and sets
    // the turret subsystem's desired yaw output
    yawController->runController(dt, turretSubsystem->getYawSetpoint());

    // run comprised command scheduler (which will run the agitator rotate command when necessary)
    comprisedCommandScheduler.run();
}

bool SentinelTurretCVCommand::isFinished() const
{
    return !pitchController->isOnline() || !yawController->isOnline();
}

void SentinelTurretCVCommand::end(bool interrupted)
{
    drivers->legacyVisionCoprocessor.stopAutoAim();
    comprisedCommandScheduler.removeCommand(&rotateAgitator, interrupted);
    turretSubsystem->setPitchMotorOutput(0);
    turretSubsystem->setYawMotorOutput(0);
}

void SentinelTurretCVCommand::updateScanningUp(
    const float motorSetpoint,
    const float minMotorSetpoint,
    const float maxMotorSetpoint,
    bool *axisScanningUp)
{
    tap::algorithms::ContiguousFloat setpointContiguous(motorSetpoint, 0, 360);

    if (abs(setpointContiguous.difference(maxMotorSetpoint)) < BOUNDS_TOLERANCE)
    {
        *axisScanningUp = false;
    }
    else if (abs(setpointContiguous.difference(minMotorSetpoint)) < BOUNDS_TOLERANCE)
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

    const float pitchSetpoint = turretSubsystem->getPitchSetpoint();

    updateScanningUp(
        pitchSetpoint,
        TurretSubsystem::PITCH_MIN_ANGLE,
        TurretSubsystem::PITCH_MAX_ANGLE,
        &pitchScanningUp);

    turretSubsystem->setPitchSetpoint(
        pitchSetpoint + (pitchScanningUp ? SCAN_DELTA_ANGLE_PITCH : -SCAN_DELTA_ANGLE_PITCH));

    const float yawSetpoint = turretSubsystem->getYawSetpoint();

    updateScanningUp(
        yawSetpoint,
        TurretSubsystem::YAW_MIN_ANGLE,
        TurretSubsystem::YAW_MAX_ANGLE,
        &yawScanningRight);

    turretSubsystem->setYawSetpoint(
        yawSetpoint + (yawScanningRight ? SCAN_DELTA_ANGLE_YAW : -SCAN_DELTA_ANGLE_YAW));
}

}  // namespace aruwsrc::control::turret::cv
