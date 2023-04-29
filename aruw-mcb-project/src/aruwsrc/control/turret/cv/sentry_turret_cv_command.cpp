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
        serial::VisionCoprocessor &visionCoprocessor,
        RobotTurretSubsystem &turretMajorSubsystem,
        RobotTurretSubsystem &turretMinorGirlbossSubsystem,
        RobotTurretSubsystem &turretMinorMalewifeSubsystem,
        algorithms::TurretYawControllerInterface &yawControllerGirlboss,
        algorithms::TurretPitchControllerInterface &pitchControllerGirlboss,
        algorithms::TurretYawControllerInterface &yawControllerMalewife,
        algorithms::TurretPitchControllerInterface &pitchControllerMalewife,
        aruwsrc::algorithms::OttoBallisticsSolver &girlbossBallisticsSolver,
        aruwsrc::algorithms::OttoBallisticsSolver &malewifeBallisticsSolver)
    : visionCoprocessor(visionCoprocessor),
    turretMajorSubsystem(turretMajorSubsystem),
      turretMinorGirlbossSubsystem(turretMinorGirlbossSubsystem),
      turretMinorMalewifeSubsystem(turretMinorMalewifeSubsystem),
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

    this->addSubsystemRequirement(turretMajorSubsystem);
    this->addSubsystemRequirement(turretMinorGirlbossSubsystem);
    this->addSubsystemRequirement(turretMinorMalewifeSubsystem);

    ignoreTargetTimeout.restart(0);
}

bool SentryTurretCVCommand::isReady() { return !isFinished(); }

void SentryTurretCVCommand::initialize()
{
    girlbossYawController->initialize();
    malewifeYawController->initialize();

    prevTime = getTimeMilliseconds();

    visionCoprocessor->sendSelectNewTargetMessage();

    enterScanMode(girlbossYawController->getSetpoint(), malewifeYawController->getSetpoint());
}

void SentryTurretCVCommand::execute()
{
    float girlbossYawSetpoint = girlbossYawController->getSetpoint();
    float malewifeYawSetpoint = malewifeYawController->getSetpoint();
    float girlbossPitchSetpoint = girlbossPitchController->getSetpoint();
    float malewifePitchSetpoint = malewifePitchController->getSetpoint();

    std::optional<OttoBallisticsSolver::BallisticsSolution> girlbossBallisticsSolution =
        ballisticsSolver->computeTurretAimAngles();
    std::optional<OttoBallisticsSolver::BallisticsSolution> malewifeBallisticsSolution =
        ballisticsSolver->computeTurretAimAngles();

    // If target spotted
    if (ignoreTargetTimeout.isExpired() && ballisticsSolution != std::nullopt)
    {
        exitScanMode();

        // Target available
        girlbossYawSetpoint = girlbossBallisticsSolution->yawAngle;
        malewifeYawSetpoint = malewifeBallisticsSolution->yawAngle;
        girlbossYawSetpoint = girlbossBallisticsSolution->pitchAngle;
        malewifeYawSetpoint = malewifeBallisticsSolution->pitchAngle;

        girlbossYawSetpoint = girlbossYawController->convertChassisAngleToControllerFrame(girlbossYawSetpoint);
        malewifeYawSetpoint = malewifeYawController->convertChassisAngleToControllerFrame(malewifeYawSetpoint);
        girlbossPitchSetpoint = girlbossPitchController->convertChassisAngleToControllerFrame(girlbossPitchSetpoint);
        malewifePitchSetpoint = malewifePitchController->convertChassisAngleToControllerFrame(malewifePitchSetpoint);

        /**
         * the setpoint returned by the ballistics solver is between [0, 2*PI)
         * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
         * that is closest to the unwrapped measured angle.
         */
        girlbossYawSetpoint = turretMinorGirlboss.yawMotor.unwrapTargetAngle(girlbossYawSetpoint);
        malewifeYawSetpoint = turretMinorMalewife.yawMotor.unwrapTargetAngle(malewifeYawSetpoint);
        girlbossPitchSetpoint = turretMinorGirlboss.pitchMotor.unwrapTargetAngle(girlbossPitchSetpoint);
        malewifePitchSetpoint = turretMinorMalewife.pitchMotor.unwrapTargetAngle(malewifePitchSetpoint);

        auto differenceWrapped = [](float measurement, float setpoint) {
            return tap::algorithms::ContiguousFloat(measurement, 0, M_TWOPI).difference(setpoint);
        };

        withinAimingTolerance = OttoBallisticsSolver::withinAimingTolerance(
            differenceWrapped(girlbossYawController->getMeasurement(), girlbossYawSetpoint),
            differenceWrapped(girlbossPitchController->getMeasurement(), girlbossPitchSetpoint),
            girlbossBallisticsSolution->distance);
        withinAimingTolerance = withinAimingTolerance && OttoBallisticsSolver::withinAimingTolerance(
            differenceWrapped(malewifeYawController->getMeasurement(), malewifeYawSetpoint),
            differenceWrapped(malewifePitchController->getMeasurement(), malewifePitchSetpoint),
            malewifeBallisticsSolution->distance);
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
        // TODO: reimplement scanning
        //       for now just holds position
        // else if (ignoreTargetTimeout.isExpired())
        // {
        //     performScanIteration(girlbossYawSetpoint, malewifeYawSetpoint);
        // }
    }

    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    // updates the turret pitch/yaw setpoint, runs the PID controller,
    // and sets the turret subsystem's desired pitch/yaw output
    girlbossPitchController->runController(dt, girlbossPitchSetpoint);
    malewifePitchController->runController(dt, malewifePitchSetpoint);
    girlbossYawController->runController(dt, girlbossYawSetpoint);
    malewifeYawController->runController(dt, malewifeYawSetpoint);
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
    float yawScanValue = girlbossYawScanner.scan();
    float yawScanValue = malewifeYawScanner.scan();

    girlbossYawScanValue = girlbossYawController->convertChassisAngleToControllerFrame(yawScanValue);
    malewifeYawScanValue = malewifeYawController->convertChassisAngleToControllerFrame(yawScanValue);

    girlbossYawSetpoint = lowPassFilter(girlbossYawSetpoint, girlbossYawScanValue, SCAN_LOW_PASS_ALPHA);
    malewifeYawSetpoint = lowPassFilter(malewifeYawSetpoint, malewifeYawScanValue, SCAN_LOW_PASS_ALPHA);
}

}  // namespace aruwsrc::control::turret::cv
