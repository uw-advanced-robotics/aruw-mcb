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

#include "turret_cv_command.hpp"

#include "tap/algorithms/ballistics.hpp"
#include "tap/architecture/clock.hpp"

#include "../algorithms/chassis_frame_turret_controller.hpp"
#include "../turret_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;

namespace aruwsrc::control::turret::cv
{
TurretCVCommand::TurretCVCommand(
    aruwsrc::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
    const float userPitchInputScalar,
    const float userYawInputScalar,
    const float defaultLaunchSpeed,
    uint8_t turretID)
    : drivers(drivers),
      turretID(turretID),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      ballisticsSolver(
          *drivers,
          odometryInterface,
          *turretSubsystem,
          frictionWheels,
          defaultLaunchSpeed,
          turretID),
      userPitchInputScalar(userPitchInputScalar),
      userYawInputScalar(userYawInputScalar)
{
    addSubsystemRequirement(turretSubsystem);
}

bool TurretCVCommand::isReady() { return !isFinished(); }

void TurretCVCommand::initialize()
{
    pitchController->initialize();
    yawController->initialize();
    prevTime = getTimeMilliseconds();
    drivers->visionCoprocessor.sendSelectNewTargetMessage();
}

void TurretCVCommand::execute()
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
        pitchSetpoint = targetPitch;
        yawSetpoint = targetYaw;

        /**
         * the setpoint returned by the ballistics solver is between [0, 2*PI)
         * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
         * that is closest to the unwrapped measured angle.
         */
        turretSubsystem->yawMotor.unwrapTargetAngle(yawSetpoint);
        turretSubsystem->pitchMotor.unwrapTargetAngle(pitchSetpoint);

        withinAimingTolerance = aruwsrc::algorithms::OttoBallisticsSolver::withinAimingTolerance(
            turretSubsystem->yawMotor.getValidChassisMeasurementError(),
            turretSubsystem->pitchMotor.getValidChassisMeasurementError(),
            targetDistance);
    }
    else
    {
        // no valid ballistics solution, let user control turret
        pitchSetpoint +=
            userPitchInputScalar * drivers->controlOperatorInterface.getTurretPitchInput(turretID);

        yawSetpoint +=
            userYawInputScalar * drivers->controlOperatorInterface.getTurretYawInput(turretID);

        withinAimingTolerance = false;
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

bool TurretCVCommand::isFinished() const
{
    return !pitchController->isOnline() || !yawController->isOnline();
}

void TurretCVCommand::end(bool)
{
    turretSubsystem->yawMotor.setMotorOutput(0);
    turretSubsystem->pitchMotor.setMotorOutput(0);
    withinAimingTolerance = false;
}

}  // namespace aruwsrc::control::turret::cv
