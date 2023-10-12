/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_turret_major_subsystem.hpp"

namespace aruwsrc::control::sentry
{
SentryTurretMajorSubsystem::SentryTurretMajorSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface* yawMotor,
    const aruwsrc::control::turret::TurretMotorConfig& yawMotorConfig)
    : tap::control::Subsystem(drivers),
      yawMotor(yawMotor, yawMotorConfig)
{
}

void SentryTurretMajorSubsystem::refresh() { yawMotor.updateMotorAngle(); }

void SentryTurretMajorSubsystem::initialize() { yawMotor.initialize(); }

float SentryTurretMajorSubsystem::getChassisYaw() const
{
    return yawMotor.getChassisFrameMeasuredAngle().getValue();
}

}  // namespace aruwsrc::control::sentry
