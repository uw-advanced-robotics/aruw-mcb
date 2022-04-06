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

//#include "sentinel.hpp"

#include "aruwsrc/control/turret/cv/sentinel_turret_cv_command.hpp"

#include <cassert>

#include "tap/algorithms/ballistics.hpp"
#include "tap/architecture/clock.hpp"

#include "../turret_controller_constants.hpp"
#include "../turret_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/cv/setpoint_scanner.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;

namespace aruwsrc::control::turret::cv
{
SentinelTurretCVCommand::SentinelTurretCVCommand(
    aruwsrc::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    Command *const firingCommand,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const chassis::ChassisSubsystem &chassisSubsystem,
    const control::launcher::FrictionWheelSubsystem &frictionWheels,
    const float userPitchInputScalar,
    const float userYawInputScalar,
    const float defaultLaunchSpeed)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      firingCommand(firingCommand),
      ballisticsSolver(
          *drivers,
          odometryInterface,
          chassisSubsystem,
          *turretSubsystem,
          frictionWheels,
          defaultLaunchSpeed),
      userPitchInputScalar(userPitchInputScalar),
      userYawInputScalar(userYawInputScalar),
      chassisSubsystem(chassisSubsystem),
      pitchScanner(
          TurretSubsystem::PITCH_MIN_ANGLE,
          TurretSubsystem::PITCH_MAX_ANGLE,
          SCAN_DELTA_ANGLE),
      yawScanner(TurretSubsystem::YAW_MIN_ANGLE, TurretSubsystem::YAW_MAX_ANGLE, SCAN_DELTA_ANGLE)
{
    assert(firingCommand != nullptr);
    assert(turretSubsystem != nullptr);
    addSubsystemRequirement(turretSubsystem);
}

bool SentinelTurretCVCommand::isReady() { return !isFinished(); }

void SentinelTurretCVCommand::initialize()
{
    pitchController->initialize();
    yawController->initialize();
    prevTime = getTimeMilliseconds();
    drivers->visionCoprocessor.sendSelectNewTargetMessage();
}

bool updateScanningDirection(
    const float motorSetpoint,
    const float minMotorSetpoint,
    const float maxMotorSetpoint,
    const float boundsTolerance,
    bool scanningPositive)
{
    tap::algorithms::ContiguousFloat setpointContiguous(motorSetpoint, 0, 360);

    bool retVal = scanningPositive;

    if (scanningPositive && abs(setpointContiguous.difference(maxMotorSetpoint)) < boundsTolerance)
    {
        retVal = false;
    }
    else if (
        !scanningPositive && abs(setpointContiguous.difference(minMotorSetpoint)) < boundsTolerance)
    {
        retVal = true;
    }

    return retVal;
}

void SentinelTurretCVCommand::execute()
{
    float pitchSetpoint = pitchController->getSetpoint();
    float yawSetpoint = yawController->getSetpoint();

    BallisticsResult ballisticsResult = getBallisticsResult();

    if (ballisticsResult.targetingAvailable)
    {
        // Target available
        pitchSetpoint = ballisticsResult.targetPitch;
        yawSetpoint = ballisticsResult.targetYaw;

        // Check if we are aiming within tolerance, if so fire
        /// TODO: This should be updated to be smarter at some point. Ideally CV sends some score
        /// to indicate whether it's worth firing at
        if (compareFloatClose(
                turretSubsystem->getCurrentPitchValue().getValue(),
                pitchSetpoint,
                FIRING_TOLERANCE) &&
            compareFloatClose(
                turretSubsystem->getCurrentYawValue().getValue(),
                yawSetpoint,
                FIRING_TOLERANCE))
        {
            // Do not re-add command if it's already scheduled as that would interrupt it
            if (!drivers->commandScheduler.isCommandScheduled(firingCommand))
            {
                drivers->commandScheduler.addCommand(firingCommand);
            }
        }
    }
    else
    {
        // Target unavailable

        // See how recently we lost target
        if (lostTargetCounter < AIM_LOST_NUM_COUNTS)
        {
            // We recently had a target. Don't start scanning yet
            lostTargetCounter++;
            // Pitch and yaw setpoint already at reasonable default value
            // by this point
        }
        else
        {
            pitchSetpoint = pitchScanner.scan(pitchSetpoint);
            yawSetpoint = yawScanner.scan(yawSetpoint);
        }
    }

    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    // updates the turret pitch setpoint based on either CV or user input, runs the PID controller,
    // and sets the turret subsystem's desired pitch output
    pitchController->runController(dt, pitchSetpoint);

    // updates the turret yaw setpoint based on either CV or user input, runs the PID controller,
    // and sets the turret subsystem's desired yaw output
    yawController->runController(dt, yawSetpoint);
}

bool SentinelTurretCVCommand::isFinished() const
{
    return !pitchController->isOnline() || !yawController->isOnline();
}

void SentinelTurretCVCommand::end(bool)
{
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
}

inline BallisticsResult SentinelTurretCVCommand::getBallisticsResult()
{
    BallisticsResult result;

    // NOTE: ballisticsSolver.computeTurretAimAngles returns values through two output params
    if (drivers->visionCoprocessor.isCvOnline() &&
        drivers->visionCoprocessor.getLastAimData().hasTarget &&
        ballisticsSolver.computeTurretAimAngles(&result.targetPitch, &result.targetYaw))
    {
        result.targetingAvailable = true;
    }
    else
    {
        result.targetingAvailable = false;
    }

    return result;
}

}  // namespace aruwsrc::control::turret::cv
