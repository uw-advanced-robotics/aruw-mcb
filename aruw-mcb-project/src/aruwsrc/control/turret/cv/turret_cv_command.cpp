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

#include "tap/architecture/clock.hpp"

#include "../turret_controller_constants.hpp"
#include "../turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::arch::clock;

namespace aruwsrc::control::turret::cv
{
TurretCVCommand::TurretCVCommand(
    aruwsrc::Drivers *drivers,
    TurretSubsystem *turretSubsystem,
    algorithms::TurretYawControllerInterface *yawController,
    algorithms::TurretPitchControllerInterface *pitchController)
    : drivers(drivers),
      turretSubsystem(turretSubsystem),
      yawController(yawController),
      pitchController(pitchController)
{
    addSubsystemRequirement(turretSubsystem);
}

bool TurretCVCommand::isReady() { return !isFinished(); }

void TurretCVCommand::initialize()
{
    drivers->legacyVisionCoprocessor.beginAutoAim();
    pitchController->initialize();
    yawController->initialize();
}

void TurretCVCommand::execute()
{
    float pitchSetpoint = pitchController->getSetpoint();
    float yawSetpoint = yawController->getSetpoint();

    if (drivers->legacyVisionCoprocessor.lastAimDataValid())
    {
        const auto &cvData = drivers->legacyVisionCoprocessor.getLastAimData();
        if (cvData.hasTarget)
        {
            pitchSetpoint = cvData.pitch;
            yawSetpoint = cvData.yaw;
        }
    }

    uint32_t currTime = getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    // updates the turret pitch setpoint based on CV input, runs the PID controller, and sets
    // the turret subsystem's desired pitch output
    pitchController->runController(dt, pitchSetpoint);

    // updates the turret yaw setpoint based on CV input, runs the PID controller, and sets
    // the turret subsystem's desired yaw output
    yawController->runController(dt, yawSetpoint);
}

bool TurretCVCommand::isFinished() const
{
    return !pitchController->isOnline() || !yawController->isOnline();
}

void TurretCVCommand::end(bool)
{
    drivers->legacyVisionCoprocessor.stopAutoAim();
    turretSubsystem->setYawMotorOutput(0);
    turretSubsystem->setPitchMotorOutput(0);
}

}  // namespace aruwsrc::control::turret::cv
