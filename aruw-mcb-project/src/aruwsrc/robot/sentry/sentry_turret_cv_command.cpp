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
        aruwsrc::algorithms::OttoBallisticsSolver<aruwsrc::sentry::TurretMinorMalewifeFrame> &malewifeBallisticsSolver)
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
      yawGirlbossScanner(
          {turretMinorGirlbossSubsystem.yawMotor.getConfig().minAngle + YAW_SCAN_ANGLE_TOLERANCE_FROM_MIN_MAX,
           turretMinorGirlbossSubsystem.yawMotor.getConfig().maxAngle - YAW_SCAN_ANGLE_TOLERANCE_FROM_MIN_MAX,
           YAW_SCAN_DELTA_ANGLE}),
      yawMalewifeScanner(
          {turretMinorMalewifeSubsystem.yawMotor.getConfig().minAngle + YAW_SCAN_ANGLE_TOLERANCE_FROM_MIN_MAX,
           turretMinorMalewifeSubsystem.yawMotor.getConfig().maxAngle - YAW_SCAN_ANGLE_TOLERANCE_FROM_MIN_MAX,
           YAW_SCAN_DELTA_ANGLE})
{

    this->addSubsystemRequirement(&turretMajorSubsystem);
    this->addSubsystemRequirement(&turretMinorGirlbossSubsystem);
    this->addSubsystemRequirement(&turretMinorMalewifeSubsystem);

    ignoreTargetTimeout.restart(0);
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
    float majorSetpoint = yawControllerMajor.getSetpoint();
    float girlbossYawSetpoint = yawControllerGirlboss.getSetpoint();
    float malewifeYawSetpoint = yawControllerMalewife.getSetpoint();
    float girlbossPitchSetpoint = pitchControllerGirlboss.getSetpoint();
    float malewifePitchSetpoint = pitchControllerMalewife.getSetpoint();

    auto girlbossBallisticsSolution = girlbossBallisticsSolver.computeTurretAimAngles();
    // std::optional<OttoBallisticsSolver::BallisticsSolution> malewifeBallisticsSolution =
    //     malewifeBallisticsSolver.computeTurretAimAngles();

    // If target spotted
    if (ignoreTargetTimeout.isExpired() && girlbossBallisticsSolution != std::nullopt)
    {
        exitScanMode();

        // Target available
        // Get world yaw setpoints
        girlbossYawSetpoint = girlbossBallisticsSolution->yawAngle;
        // malewifeYawSetpoint = malewifeBallisticsSolution->yawAngle;
        girlbossPitchSetpoint = girlbossBallisticsSolution->pitchAngle;
        // malewifeYawSetpoint = malewifeBallisticsSolution->pitchAngle;

        girlbossYawSetpoint = yawControllerGirlboss.convertChassisAngleToControllerFrame(girlbossYawSetpoint);
        // malewifeYawSetpoint = yawControllerMalewife.convertChassisAngleToControllerFrame(malewifeYawSetpoint);
        girlbossPitchSetpoint = pitchControllerGirlboss.convertChassisAngleToControllerFrame(girlbossPitchSetpoint);
        // malewifePitchSetpoint = pitchControllerMalewife.convertChassisAngleToControllerFrame(malewifePitchSetpoint);

        /**
         * the setpoint returned by the ballistics solver is between [0, 2*PI)
         * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
         * that is closest to the unwrapped measured angle.
         */
        girlbossYawSetpoint = turretMinorGirlbossSubsystem.yawMotor.unwrapTargetAngle(girlbossYawSetpoint);
        // malewifeYawSetpoint = turretMinorMalewifeSubsystem.yawMotor.unwrapTargetAngle(malewifeYawSetpoint);
        girlbossPitchSetpoint = turretMinorGirlbossSubsystem.pitchMotor.unwrapTargetAngle(girlbossPitchSetpoint);
        // malewifePitchSetpoint = turretMinorMalewifeSubsystem.pitchMotor.unwrapTargetAngle(malewifePitchSetpoint);

        auto differenceWrapped = [](float measurement, float setpoint) {
            return tap::algorithms::ContiguousFloat(measurement, 0, M_TWOPI).difference(setpoint);
        };

        withinAimingTolerance = girlbossBallisticsSolver.withinAimingTolerance(
            differenceWrapped(yawControllerGirlboss.getMeasurement(), girlbossYawSetpoint),
            differenceWrapped(pitchControllerGirlboss.getMeasurement(), girlbossPitchSetpoint),
            girlbossBallisticsSolution->distance);
        // withinAimingTolerance = withinAimingTolerance && OttoBallisticsSolver::withinAimingTolerance(
        //     differenceWrapped(yawControllerMalewife.getMeasurement(), malewifeYawSetpoint),
        //     differenceWrapped(pitchControllerMalewife.getMeasurement(), malewifePitchSetpoint),
        //     malewifeBallisticsSolution->distance);

        
    // tap::algorithms::ContiguousFloat chassisToMajor = turretMajorSubsystem.yawMotor.getChassisFrameMeasuredAngle();

    // // transform yaw by subtracting yaw of the major from the setpoint
    // tap::algorithms::ContiguousFloat wrappedGirlbossYawSetpoint(girlbossYawSetpoint - chassisToMajor.getValue(), 0, M_TWOPI);
    // tap::algorithms::ContiguousFloat wrappedMalewifeYawSetpoint(malewifeYawSetpoint - chassisToMajor.getValue(), 0, M_TWOPI);
    // }
    // else
    // {
    //     // Target unavailable
    //     withinAimingTolerance = false;

    //     // See how recently we lost target
    //     if (lostTargetCounter < AIM_LOST_NUM_COUNTS)
    //     {
    //         // We recently had a target. Don't start scanning yet
    //         lostTargetCounter++;
    //         // Pitch and yaw setpoint already at reasonable default value
    //         // by this point
    //     }
    //     // TODO: reimplement scanning
    //     //       for now just holds position
    //     // else if (ignoreTargetTimeout.isExpired())
    //     // {
    //     //     performScanIteration(girlbossYawSetpoint, malewifeYawSetpoint);
    //     // }
    // }
    }
    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    // updates the turret pitch/yaw setpoint, runs the PID controller,
    // and sets the turret subsystem's desired pitch/yaw output

    // TODO: need to transform these worldframe setpoints to major frame
    // to transform to major frame: subtract out major yaw

    // pitch needs no transformation
    pitchControllerGirlboss.runController(dt, girlbossPitchSetpoint);
    pitchControllerMalewife.runController(dt, malewifePitchSetpoint);

    // yawControllerGirlboss.runController(dt, wrappedGirlbossYawSetpoint.getValue());
    // yawControllerMalewife.runController(dt, wrappedMalewifeYawSetpoint.getValue());
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
    withinAimingTolerance = false;
}

void SentryTurretCVCommand::requestNewTarget()
{
    // TODO is there anything else the turret or firing system should do?
    visionCoprocessor.sendSelectNewTargetMessage();
}

// TODO: implement scanning
void SentryTurretCVCommand::performScanIteration(
    float &girlbossYawSetpoint, 
    float &malewifeYawSetpoint, 
    float &girlbossPitchSetpoint, 
    float &malewifePitchSetpoint)
{
    if (!scanning)
    {
        enterScanMode(girlbossYawSetpoint, malewifeYawSetpoint);
    }

    // TODO: implement linear interpolation from each turret's pitch to the scan pitch
    //       maybe something a la the setpoint scanner but just moves pitch once...
    //       seems like it deserves a less ad hoc approach
    //       or maybe just snap to it? lol it seems like that's what it does on target lock
    // TODO: implement sync between turret yaws
    float girlbossYawScanValue = yawGirlbossScanner.scan();
    float malewifeYawScanValue = yawMalewifeScanner.scan();

    girlbossYawScanValue = yawControllerGirlboss.convertChassisAngleToControllerFrame(girlbossYawScanValue);
    malewifeYawScanValue = yawControllerMalewife.convertChassisAngleToControllerFrame(malewifeYawScanValue);

    girlbossYawSetpoint = lowPassFilter(girlbossYawSetpoint, girlbossYawScanValue, SCAN_LOW_PASS_ALPHA);
    malewifeYawSetpoint = lowPassFilter(malewifeYawSetpoint, malewifeYawScanValue, SCAN_LOW_PASS_ALPHA);
}

}  // namespace aruwsrc::control::turret::cv
