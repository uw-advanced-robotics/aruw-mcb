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

#include "aruwsrc/control/turret/cv/sentinel_turret_cv_command.hpp"

#include <cassert>

#include "tap/algorithms/ballistics.hpp"
#include "tap/architecture/clock.hpp"

#include "../robot_turret_subsystem.hpp"
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
    RobotTurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    tap::control::Subsystem &launchingSubsystem,
    Command *const launchingCommand,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const float defaultLaunchSpeed,
    const uint8_t turretID)
    : ComprisedCommand(drivers),
      drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      turretID(turretID),
      launchingCommand(launchingCommand),
      ballisticsSolver(
          *drivers,
          odometryInterface,
          *turretSubsystem,
          frictionWheels,
          defaultLaunchSpeed,
          turretID),
      pitchScanner(PITCH_MIN_SCAN_ANGLE, PITCH_MAX_SCAN_ANGLE, SCAN_DELTA_ANGLE),
      yawScanner(
          turretSubsystem->yawMotor.getConfig().minAngle + YAW_SCAN_ANGLE_TOLERANCE_FROM_MIN_MAX,
          turretSubsystem->yawMotor.getConfig().maxAngle - YAW_SCAN_ANGLE_TOLERANCE_FROM_MIN_MAX,
          SCAN_DELTA_ANGLE)
{
    assert(launchingCommand != nullptr);
    assert(turretSubsystem != nullptr);
    assert(pitchController != nullptr);
    assert(yawController != nullptr);

    this->comprisedCommandScheduler.registerSubsystem(turretSubsystem);
    this->comprisedCommandScheduler.registerSubsystem(&launchingSubsystem);
    this->addSubsystemRequirement(turretSubsystem);
    this->addSubsystemRequirement(&launchingSubsystem);
}

bool SentinelTurretCVCommand::isReady() { return !isFinished(); }

void SentinelTurretCVCommand::initialize()
{
    pitchController->initialize();
    yawController->initialize();

    prevTime = getTimeMilliseconds();

    drivers->visionCoprocessor.sendSelectNewTargetMessage();

    enterScanMode(yawController->getSetpoint(), pitchController->getSetpoint());
}

void SentinelTurretCVCommand::execute()
{
    float pitchSetpoint = pitchController->getSetpoint();
    float yawSetpoint = yawController->getSetpoint();

    float targetPitch, targetYaw, targetDistance, timeOfFlight;
    bool ballisticsSolutionAvailable = ballisticsSolver.computeTurretAimAngles(
        &targetPitch,
        &targetYaw,
        &targetDistance,
        &timeOfFlight);

    if (ballisticsSolutionAvailable)
    {
        exitScanMode();

        // Target available
        pitchSetpoint = targetPitch;
        yawSetpoint = targetYaw;

        auto turretController = turretSubsystem->yawMotor.getTurretController();
        if (turretController != nullptr)
        {
            yawSetpoint = turretController->convertChassisAngleToControllerFrame(yawSetpoint);
        }

        /**
         * the setpoint returned by the ballistics solver is between [0, 2*PI)
         * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
         * that is closest to the unwrapped measured angle.
         */
        turretSubsystem->yawMotor.unwrapTargetAngle(yawSetpoint);
        turretSubsystem->pitchMotor.unwrapTargetAngle(pitchSetpoint);

        // Check if we are aiming within tolerance, if so fire
        /// TODO: This should be updated to be smarter at some point. Ideally CV sends some score
        /// to indicate whether it's worth firing at
        if (aruwsrc::algorithms::OttoBallisticsSolver::withinAimingTolerance(
                turretSubsystem->yawMotor.getValidChassisMeasurementError(),
                turretSubsystem->pitchMotor.getValidChassisMeasurementError(),
                targetDistance))
        {
            // Do not re-add command if it's already scheduled as that would interrupt it
            if (!drivers->commandScheduler.isCommandScheduled(launchingCommand))
            {
                drivers->commandScheduler.addCommand(launchingCommand);
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
            performScanIteration(yawSetpoint, pitchSetpoint);
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

    comprisedCommandScheduler.run();
}

bool SentinelTurretCVCommand::isFinished() const
{
    return !pitchController->isOnline() || !yawController->isOnline();
}

void SentinelTurretCVCommand::end(bool)
{
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
    this->comprisedCommandScheduler.removeCommand(launchingCommand, true);
}

void SentinelTurretCVCommand::requestNewTarget()
{
    // TODO is there anything else the turret or firing system should do?
    drivers->visionCoprocessor.sendSelectNewTargetMessage();
}

void SentinelTurretCVCommand::changeScanningQuadrant()
{
    // basic quadrant change for proof-of concept, if turret on left side, move right, otherwise
    // move left
    const float angleChange = copysignf(M_PI_2, -turretSubsystem->yawMotor.getAngleFromCenter());
    yawController->setSetpoint(yawController->getSetpoint() + angleChange);
}

void SentinelTurretCVCommand::performScanIteration(float &yawSetpoint, float &pitchSetpoint)
{
    if (!scanning)
    {
        enterScanMode(yawSetpoint, pitchSetpoint);
    }

    float yawScanValue = yawScanner.scan();
    float pitchScanValue = pitchScanner.scan();

    auto yawController = turretSubsystem->yawMotor.getTurretController();

    if (yawController != nullptr)
    {
        yawScanValue = yawController->convertChassisAngleToControllerFrame(yawScanValue);
    }

    yawSetpoint = lowPassFilter(yawSetpoint, yawScanValue, SCAN_LOW_PASS_ALPHA);
    pitchSetpoint = lowPassFilter(pitchSetpoint, pitchScanValue, SCAN_LOW_PASS_ALPHA);
}

}  // namespace aruwsrc::control::turret::cv
