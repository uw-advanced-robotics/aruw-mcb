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

#include "../turret_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
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
    tap::control::Subsystem &firingSubsystem,
    Command *const firingCommand,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const control::launcher::RefereeFeedbackFrictionWheelSubsystem &frictionWheels,
    const float defaultLaunchSpeed,
    const uint8_t turretID)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      turretID(turretID),
      firingCommand(firingCommand),
      ballisticsSolver(
          *drivers,
          odometryInterface,
          *turretSubsystem,
          frictionWheels,
          defaultLaunchSpeed,
          turretID),
      pitchScanner(
          turretSubsystem->pitchMotor.getConfig().minAngle,
          turretSubsystem->pitchMotor.getConfig().maxAngle,
          SCAN_DELTA_ANGLE),
      yawScanner(
          turretSubsystem->yawMotor.getConfig().minAngle,
          turretSubsystem->yawMotor.getConfig().maxAngle,
          SCAN_DELTA_ANGLE)
{
    assert(firingCommand != nullptr);
    assert(turretSubsystem != nullptr);
    assert(pitchController != nullptr);
    assert(yawController != nullptr);

    addSubsystemRequirement(turretSubsystem);
    addSubsystemRequirement(&firingSubsystem);
}

bool SentinelTurretCVCommand::isReady() { return !isFinished(); }

void SentinelTurretCVCommand::initialize()
{
    pitchController->initialize();
    yawController->initialize();
    prevTime = getTimeMilliseconds();
    drivers->visionCoprocessor.sendSelectNewTargetMessage();
}

void SentinelTurretCVCommand::execute()
{
    float pitchSetpoint = pitchController->getSetpoint();
    float yawSetpoint = yawController->getSetpoint();

    float targetPitch;
    float targetYaw;
    bool ballisticsSolutionAvailable =
        ballisticsSolver.computeTurretAimAngles(&targetPitch, &targetYaw);

    if (ballisticsSolutionAvailable)
    {
        // Target available
        pitchSetpoint = targetPitch;
        yawSetpoint = targetYaw;

        // Check if we are aiming within tolerance, if so fire
        /// TODO: This should be updated to be smarter at some point. Ideally CV sends some score
        /// to indicate whether it's worth firing at
        if (compareFloatClose(
                turretSubsystem->pitchMotor.getChassisFrameMeasuredAngle().getValue(),
                pitchSetpoint,
                FIRING_TOLERANCE) &&
            compareFloatClose(
                turretSubsystem->yawMotor.getChassisFrameMeasuredAngle().getValue(),
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
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
}

}  // namespace aruwsrc::control::turret::cv
