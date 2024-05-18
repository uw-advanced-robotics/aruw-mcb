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

#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
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
    aruwsrc::algorithms::OttoBallisticsSolver::BallisticsSolution &solution,
    float *desiredYawSetpoint,
    float *desiredPitchSetpoint,
    bool *withinAimingTolerance)
{
    // Get world-relative setpoints
    *desiredYawSetpoint = solution.yawAngle;
    *desiredPitchSetpoint = solution.pitchAngle;

    // convert world-relative setpoints to turret major frame

    /**
     * the setpoint returned by the ballistics solver is between [0, 2*PI)
     * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
     * that is closest to the unwrapped measured angle.
     */
    *desiredYawSetpoint = config.turretSubsystem.yawMotor.unwrapTargetAngle(*desiredYawSetpoint);
    *desiredPitchSetpoint =
        config.turretSubsystem.pitchMotor.unwrapTargetAngle(*desiredPitchSetpoint);

    auto differenceWrapped = [](float measurement, float setpoint)
    { return tap::algorithms::WrappedFloat(measurement, 0, M_TWOPI).minDifference(setpoint); };

    *withinAimingTolerance = turretLeftConfig.ballisticsSolver.withinAimingTolerance(
        differenceWrapped(config.yawController.getMeasurement(), *desiredYawSetpoint),
        differenceWrapped(config.pitchController.getMeasurement(), *desiredPitchSetpoint),
        solution.distance);
}

float debug_majorSetpoint;
float debug_leftPitchSetpoint;
float debug_rightPitchSetpoint;
float debug_leftYawSetpoint;
float debug_rightYawSetpoint;
bool debug = false;

float majorSetpoint;
float leftYawSetpoint;
float rightYawSetpoint;
float leftPitchSetpoint;
float rightPitchSetpoint;
void SentryTurretCVCommand::execute()
{
    // setpoints are in chassis frame
    majorSetpoint = yawControllerMajor.getSetpoint();
    leftYawSetpoint = turretLeftConfig.yawController.getSetpoint();
    rightYawSetpoint = turretRightConfig.yawController.getSetpoint();
    leftPitchSetpoint = turretLeftConfig.pitchController.getSetpoint();
    rightPitchSetpoint = turretRightConfig.pitchController.getSetpoint();

    auto leftBallisticsSolution = turretLeftConfig.ballisticsSolver.computeTurretAimAngles();
    auto rightBallisticsSolution = turretRightConfig.ballisticsSolver.computeTurretAimAngles();

    targetFound = visionCoprocessor.isCvOnline() && !(leftBallisticsSolution == std::nullopt &&
                                                      rightBallisticsSolution == std::nullopt);

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

        // sus: one of these could be std::nullopt ?
        WrappedFloat leftYawWrapped(leftBallisticsSolution->yawAngle, 0, M_TWOPI);
        WrappedFloat rightYawWrapped(rightBallisticsSolution->yawAngle, 0, M_TWOPI);

        // major averaging
        // @todo use interpolate method
        WrappedFloat majorYawWrapped(leftYawWrapped.minDifference(rightYawWrapped), -M_PI, M_PI);

        majorYawWrapped = WrappedFloat(majorYawWrapped.getWrappedValue() / -2.0f, 0, M_TWOPI);
        majorYawWrapped += rightYawWrapped;

        majorSetpoint = majorYawWrapped.getWrappedValue();
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

            majorScanValue += WrappedFloat(YAW_SCAN_DELTA_ANGLE * scanDir, 0.0f, M_TWOPI);
            majorSetpoint =
                lowPassFilter(majorSetpoint, majorScanValue.getWrappedValue(), SCAN_LOW_PASS_ALPHA);
            leftPitchSetpoint = SCAN_TURRET_MINOR_PITCH;
            rightPitchSetpoint = SCAN_TURRET_MINOR_PITCH;

            // temp = WrappedFloat(sentryTransforms.getWorldToTurretMajor().getYaw(), 0.0f, M_TWOPI)
            //            .getWrappedValue();

            leftYawSetpoint = SCAN_TURRET_LEFT_YAW + majorSetpoint;
            leftYawSetpoint =
                turretLeftConfig.turretSubsystem.yawMotor.unwrapTargetAngle(leftYawSetpoint);
            rightYawSetpoint = SCAN_TURRET_RIGHT_YAW + majorSetpoint;
            rightYawSetpoint =
                turretRightConfig.turretSubsystem.yawMotor.unwrapTargetAngle(rightYawSetpoint);
        }
    }

    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    if (debug)
    {
        // yawControllerMajor.runController(dt, debug_majorSetpoint);

        turretLeftConfig.pitchController.runController(dt, debug_leftPitchSetpoint);
        turretRightConfig.pitchController.runController(dt, debug_rightPitchSetpoint);

        turretLeftConfig.yawController.runController(dt, debug_leftYawSetpoint);
        turretRightConfig.yawController.runController(dt, debug_rightYawSetpoint);
    }
    else
    {
        // yawControllerMajor.runController(dt, majorSetpoint);

        turretLeftConfig.pitchController.runController(dt, leftPitchSetpoint);
        turretRightConfig.pitchController.runController(dt, rightPitchSetpoint);

        turretLeftConfig.yawController.runController(dt, leftYawSetpoint);
        turretRightConfig.yawController.runController(dt, rightYawSetpoint);
    }
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
