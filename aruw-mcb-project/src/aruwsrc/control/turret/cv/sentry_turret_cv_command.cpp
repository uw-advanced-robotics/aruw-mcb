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
#include "sentry_turret_cv_command.hpp"

#include <cassert>

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/cv/setpoint_scanner.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;
using namespace aruwsrc::algorithms;

namespace aruwsrc::control::sentry
{
SentryTurretCVCommand::SentryTurretCVCommand(
    serial::VisionCoprocessor &visionCoprocessor,
    aruwsrc::control::turret::YawTurretSubsystem &turretMajorSubsystem,
    aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMajor,
    TurretConfig &turretLeftConfig,
    TurretConfig &turretRightConfig,
    aruwsrc::sentry::SentryTransforms &sentryTransforms)
    : visionCoprocessor(visionCoprocessor),
      turretMajorSubsystem(turretMajorSubsystem),
      yawControllerMajor(yawControllerMajor),
      turretLeftConfig(turretLeftConfig),
      turretRightConfig(turretRightConfig),
      sentryTransforms(sentryTransforms)
{
    this->addSubsystemRequirement(&turretMajorSubsystem);
    this->addSubsystemRequirement(&turretLeftConfig.turretSubsystem);
    this->addSubsystemRequirement(&turretRightConfig.turretSubsystem);
}

bool SentryTurretCVCommand::isReady() { return !isFinished(); }

void SentryTurretCVCommand::initialize()
{
    prevTime = getTimeMilliseconds();
    visionCoprocessor.sendSelectNewTargetMessage();
}

void SentryTurretCVCommand::computeAimSetpoints(
    TurretConfig &config,
    aruwsrc::sentry::SentryBallisticsSolver::BallisticsSolution &solution,
    WrappedFloat *desiredYawSetpoint,
    WrappedFloat *desiredPitchSetpoint,
    bool *withinAimingTolerance)
{
    // Get world-relative setpoints
    *desiredYawSetpoint = Angle(solution.yawAngle);
    *desiredPitchSetpoint = Angle(solution.pitchAngle);

    *withinAimingTolerance = turretLeftConfig.ballisticsSolver.withinAimingTolerance(
        config.yawController.getMeasurement().minDifference(*desiredYawSetpoint),
        config.pitchController.getMeasurement().minDifference(*desiredPitchSetpoint),
        solution.distance);
}

void SentryTurretCVCommand::execute()
{
    // setpoints are in chassis frame
    WrappedFloat majorSetpoint = yawControllerMajor.getSetpoint();
    WrappedFloat leftYawSetpoint = turretLeftConfig.yawController.getSetpoint();
    WrappedFloat rightYawSetpoint = turretRightConfig.yawController.getSetpoint();
    WrappedFloat leftPitchSetpoint = turretLeftConfig.pitchController.getSetpoint();
    WrappedFloat rightPitchSetpoint = turretRightConfig.pitchController.getSetpoint();

    auto leftBallisticsSolution = turretLeftConfig.ballisticsSolver.computeTurretAimAngles();
    auto rightBallisticsSolution = turretRightConfig.ballisticsSolver.computeTurretAimAngles();

    // @todo: does not allow for independent turret aiming
    targetFound =
        (leftBallisticsSolution != std::nullopt && rightBallisticsSolution != std::nullopt);

    // Turret minor control
    // If target spotted
    if (targetFound)
    {
        exitScanMode();

        if (leftBallisticsSolution != std::nullopt)
        {
            computeAimSetpoints(
                turretLeftConfig,
                leftBallisticsSolution.value(),
                &leftYawSetpoint,
                &leftPitchSetpoint,
                &withinAimingToleranceLeft);
        }

        if (rightBallisticsSolution != std::nullopt)
        {
            computeAimSetpoints(
                turretRightConfig,
                rightBallisticsSolution.value(),
                &rightYawSetpoint,
                &rightPitchSetpoint,
                &withinAimingToleranceRight);
        }

        // major averaging
        majorSetpoint = leftYawSetpoint.minInterpolate(rightYawSetpoint, 0.5);
    }
    else
    {
        withinAimingToleranceLeft = false;
        withinAimingToleranceRight = false;

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
            // Scan
            if (!scanning)
            {
                enterScanMode(majorSetpoint);
            }

            // scan logic: start at some default, scan 180deg clockwise, change direction
            // scan 180 ccw, change, etc.
            float v = majorScanValue.getWrappedValue();
            if (v >= CCW_TO_CW_WRAP_VALUE)
                scanDir = SCAN_CLOCKWISE;  // decreases angle
            else if (v <= CW_TO_CCW_WRAP_VALUE)
                scanDir = SCAN_COUNTER_CLOCKWISE;  // increases angle

            majorScanValue += YAW_SCAN_DELTA_ANGLE * scanDir;
            majorSetpoint = majorSetpoint.minInterpolate(
                majorScanValue,
                SCAN_LOW_PASS_ALPHA);  // lowpass filter

            leftPitchSetpoint = Angle(SCAN_TURRET_MINOR_PITCH);
            rightPitchSetpoint = Angle(SCAN_TURRET_MINOR_PITCH);

            leftYawSetpoint = majorSetpoint + SCAN_TURRET_LEFT_YAW;
            rightYawSetpoint = majorSetpoint + SCAN_TURRET_RIGHT_YAW;
        }
    }

    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    yawControllerMajor.runController(dt, majorSetpoint);

    turretLeftConfig.pitchController.runController(dt, leftPitchSetpoint);
    turretRightConfig.pitchController.runController(dt, rightPitchSetpoint);

    turretLeftConfig.yawController.runController(dt, leftYawSetpoint);
    turretRightConfig.yawController.runController(dt, rightYawSetpoint);
}

bool SentryTurretCVCommand::isFinished() const
{
    return !turretLeftConfig.pitchController.isOnline() ||
           !turretLeftConfig.yawController.isOnline() ||
           !turretRightConfig.pitchController.isOnline() ||
           !turretRightConfig.yawController.isOnline();
}

void SentryTurretCVCommand::end(bool)
{
    turretMajorSubsystem.getMutableMotor().setMotorOutput(0);

    turretLeftConfig.turretSubsystem.pitchMotor.setMotorOutput(0);
    turretRightConfig.turretSubsystem.pitchMotor.setMotorOutput(0);

    turretLeftConfig.turretSubsystem.yawMotor.setMotorOutput(0);
    turretRightConfig.turretSubsystem.yawMotor.setMotorOutput(0);

    withinAimingToleranceLeft = false;
    withinAimingToleranceRight = false;
    exitScanMode();
}

void SentryTurretCVCommand::requestNewTarget() { visionCoprocessor.sendSelectNewTargetMessage(); }

}  // namespace aruwsrc::control::sentry
