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

#include "turret_major_control_command.hpp"

#include "tap/drivers.hpp"

#include "aruwsrc/robot/sentry/sentry_control_operator_interface.hpp"

namespace aruwsrc::control::turret::sentry
{
TurretMajorSentryControlCommand::TurretMajorSentryControlCommand(
    tap::Drivers *drivers,
    SentryControlOperatorInterface &controlOperatorInterface,
    YawTurretSubsystem &turretMajorSubsystem,
    algorithms::TurretYawControllerInterface &yawController,
    float userYawInputScalar)
    : drivers(drivers),
      controlOperatorInterface(controlOperatorInterface),
      turretMajorSubsystem(turretMajorSubsystem),
      yawController(yawController),
      userYawInputScalar(userYawInputScalar)
{
    addSubsystemRequirement(&turretMajorSubsystem);
}

bool TurretMajorSentryControlCommand::isReady() { return !isFinished(); }

void TurretMajorSentryControlCommand::initialize()
{
    yawController.initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void TurretMajorSentryControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    const float yawSetpoint =
        yawController.getSetpoint() +
        userYawInputScalar * controlOperatorInterface.getTurretMajorYawVelocity();

    lastYawSetPoint = yawSetpoint;

    yawController.runController(dt, yawSetpoint);
}

bool TurretMajorSentryControlCommand::isFinished() const { return !yawController.isOnline(); }

void TurretMajorSentryControlCommand::end(bool)
{
    turretMajorSubsystem.getMutableMotor().setMotorOutput(0);
}

}  // namespace aruwsrc::control::turret::sentry
