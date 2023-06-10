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

#include "tap/algorithms/contiguous_float.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;
using namespace aruwsrc::algorithms;

namespace aruwsrc::control::turret
{
SentryTurretCVCommand::SentryTurretCVCommand(
        serial::VisionCoprocessor &visionCoprocessor,
        aruwsrc::control::turret::SentryTurretMajorSubsystem &turretMajorSubsystem,
        aruwsrc::control::turret::SentryTurretMinorSubsystem &turretMinorGirlbossSubsystem,
        aruwsrc::control::turret::SentryTurretMinorSubsystem &turretMinorMalewifeSubsystem,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMajor,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerGirlboss,
        aruwsrc::control::turret::algorithms::TurretPitchControllerInterface &pitchControllerGirlboss,
        aruwsrc::control::turret::algorithms::TurretYawControllerInterface &yawControllerMalewife,
        aruwsrc::control::turret::algorithms::TurretPitchControllerInterface &pitchControllerMalewife,
        aruwsrc::algorithms::OttoBallisticsSolver<aruwsrc::sentry::TurretMinorGirlbossFrame> &girlbossBallisticsSolver,
        aruwsrc::algorithms::OttoBallisticsSolver<aruwsrc::sentry::TurretMinorMalewifeFrame> &malewifeBallisticsSolver,
        aruwsrc::sentry::SentryTransforms &sentryTransforms)
    : visionCoprocessor(visionCoprocessor),
      turretMajorSubsystem(turretMajorSubsystem),
      turretMinorGirlbossSubsystem(turretMinorGirlbossSubsystem),
      turretMinorMalewifeSubsystem(turretMinorMalewifeSubsystem),
      yawControllerMajor(yawControllerMajor),
      yawControllerGirlboss(yawControllerGirlboss),
      pitchControllerGirlboss(pitchControllerGirlboss),
      yawControllerMalewife(yawControllerMalewife),
      pitchControllerMalewife(pitchControllerMalewife),
      girlbossBallisticsSolver(girlbossBallisticsSolver),
      malewifeBallisticsSolver(malewifeBallisticsSolver),
      sentryTransforms(sentryTransforms)
{

    this->addSubsystemRequirement(&turretMajorSubsystem);
    this->addSubsystemRequirement(&turretMinorGirlbossSubsystem);
    this->addSubsystemRequirement(&turretMinorMalewifeSubsystem);
}

bool SentryTurretCVCommand::isReady() { return !isFinished(); }

void SentryTurretCVCommand::initialize()
{
    yawControllerGirlboss.initialize();
    yawControllerMalewife.initialize();

    prevTime = getTimeMilliseconds();

    visionCoprocessor.sendSelectNewTargetMessage();

    // enterScanMode(yawControllerGirlboss.getSetpoint(), yawControllerMalewife.getSetpoint());
}

void SentryTurretCVCommand::execute()
{
    // setpoints are in chassis frame
    float majorSetpoint = yawControllerMajor.getSetpoint();
    float girlbossYawSetpoint = yawControllerGirlboss.getSetpoint();
    float malewifeYawSetpoint = yawControllerMalewife.getSetpoint();
    float girlbossPitchSetpoint = pitchControllerGirlboss.getSetpoint();
    float malewifePitchSetpoint = pitchControllerMalewife.getSetpoint();

    // world angles
    auto girlbossAimData = visionCoprocessor.getLastAimData(0);
    auto girlbossBallisticsSolution = girlbossBallisticsSolver.computeTurretAimAngles(girlbossAimData);

    auto malewifeAimData = visionCoprocessor.getLastAimData(1);
    auto malewifeBallisticsSolution = malewifeBallisticsSolver.computeTurretAimAngles(malewifeAimData);

    bool targetFound = visionCoprocessor.isCvOnline() && !(girlbossBallisticsSolution == std::nullopt && malewifeBallisticsSolution == std::nullopt);

    // Turret minor control
    // If target spotted
    if (targetFound)
    {
        exitScanMode();

        // Get world-relative setpoints
        girlbossYawSetpoint = girlbossBallisticsSolution->yawAngle;
        girlbossPitchSetpoint = girlbossBallisticsSolution->pitchAngle;


        // convert world-relative setpoints to turret major frame setpoint
        girlbossYawSetpoint = girlbossYawSetpoint - sentryTransforms.getWorldToTurretMajor().getYaw();
        girlbossPitchSetpoint = girlbossPitchSetpoint - sentryTransforms.getWorldToTurretMajor().getPitch();

        /**
         * the setpoint returned by the ballistics solver is between [0, 2*PI)
         * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
         * that is closest to the unwrapped measured angle.
         */
        girlbossYawSetpoint = turretMinorGirlbossSubsystem.yawMotor.unwrapTargetAngle(girlbossYawSetpoint);
        girlbossPitchSetpoint = turretMinorGirlbossSubsystem.pitchMotor.unwrapTargetAngle(girlbossPitchSetpoint);

        auto differenceWrappedGirlboss = [](float measurement, float setpoint) {
            return tap::algorithms::ContiguousFloat(measurement, 0, M_TWOPI).difference(setpoint);
        };

        withinAimingToleranceGirlboss = girlbossBallisticsSolver.withinAimingTolerance(
            differenceWrappedGirlboss(yawControllerGirlboss.getMeasurement(), girlbossYawSetpoint),
            differenceWrappedGirlboss(pitchControllerGirlboss.getMeasurement(), girlbossPitchSetpoint),
            girlbossBallisticsSolution->distance);

        // Get world-relative setpoints
        malewifeYawSetpoint = malewifeBallisticsSolution->yawAngle;
        malewifePitchSetpoint = malewifeBallisticsSolution->pitchAngle;


        // convert world-relative setpoints to respective turret frame setpoint
        // hold on, is it minor frame or major frame that the controllers are running in?
        // convert world-relative setpoints to turret major frame setpoint
        malewifeYawSetpoint = malewifeYawSetpoint - sentryTransforms.getWorldToTurretMajor().getYaw();
        malewifePitchSetpoint = malewifePitchSetpoint - sentryTransforms.getWorldToTurretMajor().getPitch();

        /**
         * the setpoint returned by the ballistics solver is between [0, 2*PI)
         * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
         * that is closest to the unwrapped measured angle.
         */
        malewifeYawSetpoint = turretMinorMalewifeSubsystem.yawMotor.unwrapTargetAngle(malewifeYawSetpoint);
        malewifePitchSetpoint = turretMinorMalewifeSubsystem.pitchMotor.unwrapTargetAngle(malewifePitchSetpoint);

        auto differenceWrappedMalewife = [](float measurement, float setpoint) {
            return tap::algorithms::ContiguousFloat(measurement, 0, M_TWOPI).difference(setpoint);
        };

        withinAimingToleranceMalewife = malewifeBallisticsSolver.withinAimingTolerance(
            differenceWrappedMalewife(yawControllerMalewife.getMeasurement(), malewifeYawSetpoint),
            differenceWrappedMalewife(pitchControllerMalewife.getMeasurement(), malewifePitchSetpoint),
            malewifeBallisticsSolution->distance);

        WrappedFloat girlbossYawWrapped(girlbossBallisticsSolution->yawAngle, 0, M_TWOPI);
        WrappedFloat malewifeYawWrapped(malewifeBallisticsSolution->yawAngle, 0, M_TWOPI);

        // major averaging
        // majorSetpoint = ((girlbossBallisticsSolution->yawAngle + malewifeBallisticsSolution->yawAngle) / 2.0f);

        auto& worldToChassisTransform = sentryTransforms.getWorldToChassis();
        WrappedFloat majorYawWrapped = girlbossYawWrapped.minDifference(malewifeYawWrapped);

        debug1 = majorYawWrapped;
        debug2 = girlbossYawWrapped;
        debug3 = malewifeYawWrapped;

        majorYawWrapped /= -2.0f;
        majorYawWrapped += malewifeYawWrapped;

        majorSetpoint = majorYawWrapped.getValue();
    }
    else
    {
        withinAimingToleranceGirlboss = false;
        withinAimingToleranceMalewife = false;

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
            majorScanValue += WrappedFloat(YAW_SCAN_DELTA_ANGLE, 0.0f, M_TWOPI);
            // lowPassFilter(majorSetpoint, majorScanValue, SCAN_LOW_PASS_ALPHA);
            majorSetpoint = majorScanValue.getValue();
            girlbossPitchSetpoint = SCAN_TURRET_MINOR_PITCH;
            malewifePitchSetpoint = SCAN_TURRET_MINOR_PITCH;
            girlbossYawSetpoint = SCAN_GIRLBOSS_YAW;
            malewifeYawSetpoint = SCAN_MALEWIFE_YAW;
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

    pitchControllerGirlboss.runController(dt, girlbossPitchSetpoint);
    pitchControllerMalewife.runController(dt, malewifePitchSetpoint);

    yawControllerGirlboss.runController(dt, girlbossYawSetpoint);
    yawControllerMalewife.runController(dt, malewifeYawSetpoint);
}

bool SentryTurretCVCommand::isFinished() const
{
    return !yawControllerGirlboss.isOnline() || !pitchControllerGirlboss.isOnline() || !yawControllerMalewife.isOnline() || !pitchControllerMalewife.isOnline();
}

void SentryTurretCVCommand::end(bool)
{
    turretMajorSubsystem.yawMotor.setMotorOutput(0);
    turretMinorGirlbossSubsystem.pitchMotor.setMotorOutput(0);
    turretMinorMalewifeSubsystem.pitchMotor.setMotorOutput(0);
    withinAimingToleranceGirlboss = false;
    withinAimingToleranceMalewife = false;
    exitScanMode();
}

void SentryTurretCVCommand::requestNewTarget()
{
    visionCoprocessor.sendSelectNewTargetMessage();
}

}  // namespace aruwsrc::control::turret::cv
