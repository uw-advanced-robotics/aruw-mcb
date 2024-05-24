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

#include "aruwsrc/control/turret/cv/sentry_turret_cv_command.hpp"

#include <cassert>

#include "tap/algorithms/ballistics.hpp"
#include "tap/architecture/clock.hpp"

#include "../robot_turret_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/cv/setpoint_scanner.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;
using namespace aruwsrc::algorithms;

namespace aruwsrc::control::turret::cv
{
SentryTurretCVCommand::SentryTurretCVCommand(
    serial::VisionCoprocessor *visionCoprocessor,
    RobotTurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    aruwsrc::algorithms::OttoBallisticsSolver *ballisticsSolver,
    const uint8_t turretID)
    : visionCoprocessor(visionCoprocessor),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      turretID(turretID),
      ballisticsSolver(ballisticsSolver),
      pitchScanner({PITCH_MIN_SCAN_ANGLE, PITCH_MAX_SCAN_ANGLE, PITCH_SCAN_DELTA_ANGLE}),
      yawScanner(
          {turretSubsystem->yawMotor.getConfig().minAngle + YAW_SCAN_ANGLE_TOLERANCE_FROM_MIN_MAX,
           turretSubsystem->yawMotor.getConfig().maxAngle - YAW_SCAN_ANGLE_TOLERANCE_FROM_MIN_MAX,
           YAW_SCAN_DELTA_ANGLE})
{
    assert(turretSubsystem != nullptr);
    assert(pitchController != nullptr);
    assert(yawController != nullptr);
    assert(ballisticsSolver != nullptr);

    this->addSubsystemRequirement(turretSubsystem);

    ignoreTargetTimeout.restart(0);
}

bool SentryTurretCVCommand::isReady() { return !isFinished(); }

void SentryTurretCVCommand::initialize()
{
    pitchController->initialize();
    yawController->initialize();

    prevTime = getTimeMilliseconds();

    visionCoprocessor->sendSelectNewTargetMessage();

    enterScanMode(yawController->getSetpoint(), pitchController->getSetpoint());
}

void SentryTurretCVCommand::execute()
{
    float pitchSetpoint = pitchController->getSetpoint();
    float yawSetpoint = yawController->getSetpoint();

    std::optional<OttoBallisticsSolver::BallisticsSolution> ballisticsSolution =
        ballisticsSolver->computeTurretAimAngles();

    if (ignoreTargetTimeout.isExpired() && ballisticsSolution != std::nullopt)
    {
        exitScanMode();

        // Target available
        pitchSetpoint = ballisticsSolution->pitchAngle;
        yawSetpoint = ballisticsSolution->yawAngle;

        yawSetpoint = yawController->convertChassisAngleToControllerFrame(yawSetpoint);
        pitchSetpoint = pitchController->convertChassisAngleToControllerFrame(pitchSetpoint);

        /**
         * the setpoint returned by the ballistics solver is between [0, 2*PI)
         * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
         * that is closest to the unwrapped measured angle.
         */
        yawSetpoint = turretSubsystem->yawMotor.unwrapTargetAngle(yawSetpoint);
        pitchSetpoint = turretSubsystem->pitchMotor.unwrapTargetAngle(pitchSetpoint);

        auto differenceWrapped = [](float measurement, float setpoint)
        { return tap::algorithms::WrappedFloat(measurement, 0, M_TWOPI).minDifference(setpoint); };

        withinAimingTolerance = aruwsrc::algorithms::OttoBallisticsSolver::withinAimingTolerance(
            differenceWrapped(yawController->getMeasurement(), yawSetpoint),
            differenceWrapped(pitchController->getMeasurement(), pitchSetpoint),
            ballisticsSolution->distance);
    }
    else
    {
        // Target unavailable
        withinAimingTolerance = false;

        // See how recently we lost target
        if (lostTargetCounter < AIM_LOST_NUM_COUNTS)
        {
            // We recently had a target. Don't start scanning yet
            lostTargetCounter++;
            // Pitch and yaw setpoint already at reasonable default value
            // by this point
        }
        else if (ignoreTargetTimeout.isExpired())
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
}

bool SentryTurretCVCommand::isFinished() const
{
    return !pitchController->isOnline() || !yawController->isOnline();
}

void SentryTurretCVCommand::end(bool)
{
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
    withinAimingTolerance = false;
}

void SentryTurretCVCommand::requestNewTarget()
{
    // TODO is there anything else the turret or firing system should do?
    visionCoprocessor->sendSelectNewTargetMessage();
}

void SentryTurretCVCommand::changeScanningQuadrant()
{
    float currentYawAngle = yawController->getSetpoint();

    float newSetpoint =
        tap::algorithms::WrappedFloat(currentYawAngle + M_PI, 0, M_TWOPI).getWrappedValue();
    newSetpoint = turretSubsystem->yawMotor.unwrapTargetAngle(newSetpoint);
    yawController->setSetpoint(newSetpoint);
    exitScanMode();
    ignoreTargetTimeout.restart(TIME_TO_IGNORE_TARGETS_WHILE_TURNING_AROUND_MS);
}

void SentryTurretCVCommand::performScanIteration(float &yawSetpoint, float &pitchSetpoint)
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
