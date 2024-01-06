/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "turret_minor_control_command.hpp"

#include "tap/drivers.hpp"

using namespace aruwsrc::control::sentry;

namespace aruwsrc::control::turret::sentry
{
TurretMinorSentryControlCommand::TurretMinorSentryControlCommand(
    tap::Drivers *drivers,
    SentryControlOperatorInterface &controlOperatorInterface,
    SentryTurretMinorSubsystem &turretMinorSubsystem,
    algorithms::TurretYawControllerInterface &yawController,
    algorithms::TurretPitchControllerInterface &pitchController,
    float userYawInputScalar,
    float userPitchInputScalar)
    : drivers(drivers),
      controlOperatorInterface(controlOperatorInterface),
      turretMinorSubsystem(turretMinorSubsystem),
      yawController(yawController),
      pitchController(pitchController),
      userYawInputScalar(userYawInputScalar),
      userPitchInputScalar(userPitchInputScalar)
{
    addSubsystemRequirement(&turretMinorSubsystem);
}

bool TurretMinorSentryControlCommand::isReady() { return !isFinished(); }

void TurretMinorSentryControlCommand::initialize()
{
    yawController.initialize();
    pitchController.initialize();
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void TurretMinorSentryControlCommand::execute()
{
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = currTime - prevTime;
    prevTime = currTime;

    // Get pitch input from control operator interface
    float pitchInput;
    float yawInput;
    switch (turretMinorSubsystem.getTurretID())
    {
        case SentryTurretMinorSubsystem::TurretID::TURRET_ID_ZERO:
            pitchInput = controlOperatorInterface.getTurretMinor1PitchVelocity();
            yawInput = controlOperatorInterface.getTurretMinor1YawVelocity();

            break;
        case SentryTurretMinorSubsystem::TurretID::TURRET_ID_ONE:
            pitchInput = controlOperatorInterface.getTurretMinor2PitchVelocity();
            yawInput = controlOperatorInterface.getTurretMinor2YawVelocity();
            break;
        default:
            pitchInput = 0;
            yawInput = 0;
    }
    const float pitchSetpoint = pitchController.getSetpoint() + userPitchInputScalar * pitchInput;
    pitchController.runController(dt, pitchSetpoint);

    const float yawSetpoint = yawController.getSetpoint() + userYawInputScalar * yawInput;
    yawController.runController(dt, yawSetpoint);
}

bool TurretMinorSentryControlCommand::isFinished() const
{
    return !pitchController.isOnline() && !yawController.isOnline();
}

void TurretMinorSentryControlCommand::end(bool)
{
    turretMinorSubsystem.pitchMotor.setMotorOutput(0);
    turretMinorSubsystem.yawMotor.setMotorOutput(0);
}

}  // namespace aruwsrc::control::turret::sentry
