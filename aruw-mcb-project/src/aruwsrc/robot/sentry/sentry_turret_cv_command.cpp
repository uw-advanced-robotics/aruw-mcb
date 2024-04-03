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
#include <cassert>

#include "tap/algorithms/ballistics.hpp"
#include "tap/architecture/clock.hpp"

#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/control/turret/cv/setpoint_scanner.hpp"

#include "aruwsrc/robot/sentry/sentry_turret_minor_subsystem.hpp"

#include "sentry_turret_cv_command.hpp"
#include "tap/algorithms/wrapped_float.hpp"
#include "tap/algorithms/math_user_utils.hpp"

#include "tap/algorithms/contiguous_float.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;
using namespace aruwsrc::algorithms;

namespace aruwsrc::control::turret
{
SentryTurretCVCommand::SentryTurretCVCommand(
        serial::VisionCoprocessor &visionCoprocessor,
        aruwsrc::control::turret::YawTurretSubsystem &turretMajorSubsystem,
        aruwsrc::control::sentry::SentryTurretMinorSubsystem &turretLeftSubsystem,
        aruwsrc::control::sentry::SentryTurretMinorSubsystem &turretRightSubsystem,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMajor,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerLeft,
        aruwsrc::control::turret::algorithms::TurretPitchControllerInterface &pitchControllerLeft,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerRight,
        aruwsrc::control::turret::algorithms::TurretPitchControllerInterface &pitchControllerRight,
        aruwsrc::algorithms::OttoBallisticsSolver &leftBallisticsSolver,
        aruwsrc::algorithms::OttoBallisticsSolver &rightBallisticsSolver,
        aruwsrc::sentry::SentryTransforms &sentryTransforms)
    : visionCoprocessor(visionCoprocessor),
      turretMajorSubsystem(turretMajorSubsystem),
      turretLeftSubsystem(turretLeftSubsystem),
      turretRightSubsystem(turretRightSubsystem),
      yawControllerMajor(yawControllerMajor),
      yawControllerLeft(yawControllerLeft),
      pitchControllerLeft(pitchControllerLeft),
      yawControllerRight(yawControllerRight),
      pitchControllerRight(pitchControllerRight),
      leftBallisticsSolver(leftBallisticsSolver),
      rightBallisticsSolver(rightBallisticsSolver),
      sentryTransforms(sentryTransforms)
{

    this->addSubsystemRequirement(&turretMajorSubsystem);
    this->addSubsystemRequirement(&turretLeftSubsystem);
    this->addSubsystemRequirement(&turretRightSubsystem);
}

bool SentryTurretCVCommand::isReady() { return !isFinished(); }

void SentryTurretCVCommand::initialize()
{
    yawControllerLeft.initialize();
    yawControllerRight.initialize();

    prevTime = getTimeMilliseconds();

    visionCoprocessor.sendSelectNewTargetMessage();

    // enterScanMode(yawControllerLeft.getSetpoint(), yawControllerRight.getSetpoint());
}

void SentryTurretCVCommand::execute()
{
    // setpoints are in chassis frame
    float majorSetpoint = yawControllerMajor.getSetpoint();
    float TurretLeftYawSetpoint = yawControllerLeft.getSetpoint();
    float TurretRightYawSetpoint = yawControllerRight.getSetpoint();
    float TurretLeftPitchSetpoint = pitchControllerLeft.getSetpoint();
    float TurretRightPitchSetpoint = pitchControllerRight.getSetpoint();

    // world angles
    auto TurretLeftBallisticsSolution = leftBallisticsSolver.computeTurretAimAngles();

    auto TurretRightBallisticsSolution = rightBallisticsSolver.computeTurretAimAngles();

    targetFound = visionCoprocessor.isCvOnline() && !(TurretLeftBallisticsSolution == std::nullopt && TurretRightBallisticsSolution == std::nullopt);

    // Turret minor control
    // If target spotted
    if (targetFound)
    {
        exitScanMode();

        if (TurretLeftBallisticsSolution != std::nullopt)
        {
            // Get world-relative setpoints
            TurretLeftYawSetpoint = TurretLeftBallisticsSolution->yawAngle;
            TurretLeftPitchSetpoint = TurretLeftBallisticsSolution->pitchAngle;

            // convert world-relative setpoints to turret major frame setpoint
            TurretLeftYawSetpoint = TurretLeftYawSetpoint - sentryTransforms.getWorldToTurretMajor().getYaw();
            TurretLeftPitchSetpoint = TurretLeftPitchSetpoint - sentryTransforms.getWorldToTurretMajor().getPitch();

            /**
             * the setpoint returned by the ballistics solver is between [0, 2*PI)
             * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
             * that is closest to the unwrapped measured angle.
             */
            TurretLeftYawSetpoint = turretLeftSubsystem.yawMotor.unwrapTargetAngle(TurretLeftYawSetpoint);
            TurretLeftPitchSetpoint = turretLeftSubsystem.pitchMotor.unwrapTargetAngle(TurretLeftPitchSetpoint);

            auto differenceWrappedTurretLeft = [](float measurement, float setpoint) {
                return tap::algorithms::ContiguousFloat(measurement, 0, M_TWOPI).difference(setpoint);
            };

            withinAimingToleranceLeft = leftBallisticsSolver.withinAimingTolerance(
                differenceWrappedTurretLeft(yawControllerLeft.getMeasurement(), TurretLeftYawSetpoint),
                differenceWrappedTurretLeft(pitchControllerLeft.getMeasurement(), TurretLeftPitchSetpoint),
                TurretLeftBallisticsSolution->distance);
        }

        if (TurretRightBallisticsSolution != std::nullopt)
        {
            // Get world-relative setpoints
            TurretRightYawSetpoint = TurretRightBallisticsSolution->yawAngle;
            TurretRightPitchSetpoint = TurretRightBallisticsSolution->pitchAngle;

            // convert world-relative setpoints to respective turret frame setpoint
            // hold on, is it minor frame or major frame that the controllers are running in?
            // convert world-relative setpoints to turret major frame setpoint
            TurretRightYawSetpoint = TurretRightYawSetpoint - sentryTransforms.getWorldToTurretMajor().getYaw();
            TurretRightPitchSetpoint = TurretRightPitchSetpoint - sentryTransforms.getWorldToTurretMajor().getPitch();

            /**
             * the setpoint returned by the ballistics solver is between [0, 2*PI)
             * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
             * that is closest to the unwrapped measured angle.
             */
            TurretRightYawSetpoint = turretRightSubsystem.yawMotor.unwrapTargetAngle(TurretRightYawSetpoint);
            TurretRightPitchSetpoint = turretRightSubsystem.pitchMotor.unwrapTargetAngle(TurretRightPitchSetpoint);

            auto differenceWrappedTurretRight = [](float measurement, float setpoint) {
                return tap::algorithms::ContiguousFloat(measurement, 0, M_TWOPI).difference(setpoint);
            };

            withinAimingToleranceRight = rightBallisticsSolver.withinAimingTolerance(
                differenceWrappedTurretRight(yawControllerRight.getMeasurement(), TurretRightYawSetpoint),
                differenceWrappedTurretRight(pitchControllerRight.getMeasurement(), TurretRightPitchSetpoint),
                TurretRightBallisticsSolution->distance);
        }

        WrappedFloat TurretLeftYawWrapped(TurretLeftBallisticsSolution->yawAngle, 0, M_TWOPI);
        WrappedFloat TurretRightYawWrapped(TurretRightBallisticsSolution->yawAngle, 0, M_TWOPI);

        // major averaging
        auto& worldToChassisTransform = sentryTransforms.getWorldToChassis();
        // conversion because otherwise divide by 2 doesn't work right otherwise
        WrappedFloat majorYawWrapped = WrappedFloat(TurretLeftYawWrapped.minDifference(TurretRightYawWrapped).getValue(), -M_PI, M_PI);
        majorYawWrapped /= -2.0f;
        majorYawWrapped = WrappedFloat(majorYawWrapped.getValue(), 0, M_TWOPI);
        majorYawWrapped += TurretRightYawWrapped;

        majorSetpoint = majorYawWrapped.getValue();
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
            if (!scanning) { enterScanMode(majorSetpoint); }

            // scan logic: start at some default, scan 180deg clockwise, change direction
            // scan 180 ccw, change, etc.


            // possibly use some ramping when switching directions,
            // but we're pretty slow so who cares
            float v = majorScanValue.getValue();
            if (v >= CCW_TO_CW_WRAP_VALUE)
                scanDir = SCAN_CLOCKWISE; // decreases angle
            else if (v <= CW_TO_CCW_WRAP_VALUE)
                scanDir =  SCAN_COUNTER_CLOCKWISE; // increases angle

            majorScanValue += WrappedFloat(YAW_SCAN_DELTA_ANGLE * scanDir, 0.0f, M_TWOPI);
            majorSetpoint = lowPassFilter(majorSetpoint, majorScanValue.getValue(), SCAN_LOW_PASS_ALPHA);
            // majorSetpoint = majorScanValue.getValue();
            TurretLeftPitchSetpoint = SCAN_TURRET_MINOR_PITCH;
            TurretRightPitchSetpoint = SCAN_TURRET_MINOR_PITCH;
            TurretLeftYawSetpoint = SCAN_LEFT_YAW;
            TurretRightYawSetpoint = SCAN_RIGHT_YAW;
        }
    }

    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    // updates the turret pitch/yaw setpoint, runs the PID controller,
    // and sets the turret subsystem's desired pitch/yaw output

    // TODO: need to transform these worldframe setpoints to major frame
    // to transform to major frame: subtract out major yaw

    yawControllerMajor.runController(dt, majorSetpoint);

    pitchControllerLeft.runController(dt, TurretLeftPitchSetpoint);
    pitchControllerRight.runController(dt, TurretRightPitchSetpoint);

    yawControllerLeft.runController(dt, TurretLeftYawSetpoint);
    yawControllerRight.runController(dt, TurretRightYawSetpoint);
}

bool SentryTurretCVCommand::isFinished() const
{
    return !yawControllerLeft.isOnline() || !pitchControllerLeft.isOnline() || !yawControllerRight.isOnline() || !pitchControllerRight.isOnline();
}

void SentryTurretCVCommand::end(bool)
{
    turretMajorSubsystem.refreshSafeDisconnect();
    turretLeftSubsystem.pitchMotor.setMotorOutput(0);
    turretRightSubsystem.pitchMotor.setMotorOutput(0);
    withinAimingToleranceLeft = false;
    withinAimingToleranceRight = false;
    exitScanMode();
}

void SentryTurretCVCommand::requestNewTarget()
{
    visionCoprocessor.sendSelectNewTargetMessage();
}

}  // namespace aruwsrc::control::turret::cv
