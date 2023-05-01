/*
 * Copyright (c) 2022-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/control/turret/constants/turret_constants.hpp"

namespace aruwsrc::control::turret
{
SentryTurretMajorSubsystem::SentryTurretMajorSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface* yawMotor,
    const TurretMotorConfig& yawMotorConfig)
    : Subsystem(drivers),
      yawMotor(yawMotor, yawMotorConfig)
{
}

void SentryTurretMajorSubsystem::refresh(){
    yawMotor.updateMotorAngle();
}

void SentryTurretMajorSubsystem::initialize()
{
    yawMotor.initialize();
}

uint32_t SentryTurretMajorSubsystem::getLastMeasurementTimeMicroseconds() const
{
    return tap::arch::clock::getTimeMicroseconds();
}

}  // namespace aruwsrc::control::turret
