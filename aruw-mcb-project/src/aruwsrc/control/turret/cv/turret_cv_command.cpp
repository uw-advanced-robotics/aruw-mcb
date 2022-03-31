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

#include "../turret_subsystem.hpp"
#include "aruwsrc/algorithms/odometry/otto_velocity_odometry_2d_subsystem.hpp"
#include "aruwsrc/constants.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
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
    const chassis::ChassisSubsystem &chassisSubsystem,
    const control::launcher::FrictionWheelSubsystem &frictionWheels,
    const float userPitchInputScalar,
    const float userYawInputScalar,
    const float defaultLaunchSpeed)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      ballisticsSolver(
          *drivers,
          odometryInterface,
          chassisSubsystem,
          *turretSubsystem,
          frictionWheels,
          defaultLaunchSpeed),
      userPitchInputScalar(userPitchInputScalar),
      userYawInputScalar(userYawInputScalar),
      chassisSubsystem(chassisSubsystem)
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

    if (!(drivers->visionCoprocessor.isCvOnline() &&
          drivers->visionCoprocessor.getLastAimData().hasTarget &&
          ballisticsSolver.computeTurretAimAngles(&pitchSetpoint, &yawSetpoint)))
    {
        // no valid CV data, let user control turret
        pitchSetpoint +=
            userPitchInputScalar * drivers->controlOperatorInterface.getTurretPitchInput();

        yawSetpoint += userYawInputScalar * drivers->controlOperatorInterface.getTurretYawInput();
    }
    else
    {
        pitchSetpoint = modm::toDegree(pitchSetpoint);
        yawSetpoint = modm::toDegree(yawSetpoint);
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
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
}

}  // namespace aruwsrc::control::turret::cv
