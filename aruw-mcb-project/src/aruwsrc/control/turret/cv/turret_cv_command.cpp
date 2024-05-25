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
#include "../robot_turret_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/control/launcher/referee_feedback_friction_wheel_subsystem.hpp"

using namespace tap::arch::clock;
using namespace tap::algorithms;
using namespace aruwsrc::algorithms;

namespace aruwsrc::control::turret::cv
{
TurretCVCommand::TurretCVCommand(
    serial::VisionCoprocessor *visionCoprocessor,
    control::ControlOperatorInterface *controlOperatorInterface,
    RobotTurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController,
    aruwsrc::algorithms::OttoBallisticsSolver *ballisticsSolver,
    const float userYawInputScalar,
    const float userPitchInputScalar,
    uint8_t turretID)
    : visionCoprocessor(visionCoprocessor),
      controlOperatorInterface(controlOperatorInterface),
      turretID(turretID),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      ballisticsSolver(ballisticsSolver),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar)
{
    assert(ballisticsSolver != nullptr);

    assert(turretID == ballisticsSolver->turretID);
    addSubsystemRequirement(turretSubsystem);
}

bool TurretCVCommand::isReady() { return !isFinished(); }

void TurretCVCommand::initialize()
{
    pitchController->initialize();
    yawController->initialize();
    prevTime = getTimeMilliseconds();
    visionCoprocessor->sendSelectNewTargetMessage();
}

void TurretCVCommand::execute()
{
    float pitchSetpoint = pitchController->getSetpoint();
    float yawSetpoint = yawController->getSetpoint();

    std::optional<OttoBallisticsSolver::BallisticsSolution> ballisticsSolution =
        ballisticsSolver->computeTurretAimAngles();

    if (ballisticsSolution != std::nullopt)
    {
        pitchSetpoint = ballisticsSolution->pitchAngle;
        yawSetpoint = ballisticsSolution->yawAngle;

        /**
         * the setpoint returned by the ballistics solver is between [0, 2*PI)
         * the desired setpoint is unwrapped when motor angles are limited, so find the setpoint
         * that is closest to the unwrapped measured angle.
         */
        yawSetpoint = turretSubsystem->yawMotor.unwrapTargetAngle(yawSetpoint);
        pitchSetpoint = turretSubsystem->pitchMotor.unwrapTargetAngle(pitchSetpoint);

        auto differenceWrapped = [](float measurement, float setpoint) {
            return tap::algorithms::WrappedFloat(measurement, 0, M_TWOPI).minDifference(setpoint);
        };

        withinAimingTolerance = aruwsrc::algorithms::OttoBallisticsSolver::withinAimingTolerance(
            differenceWrapped(yawController->getMeasurement(), yawSetpoint),
            differenceWrapped(pitchController->getMeasurement(), pitchSetpoint),
            ballisticsSolution->distance);
    }
    else
    {
        // no valid ballistics solution, let user control turret
        pitchSetpoint +=
            userPitchInputScalar * controlOperatorInterface->getTurretPitchInput(turretID);

        yawSetpoint += userYawInputScalar * controlOperatorInterface->getTurretYawInput(turretID);

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
