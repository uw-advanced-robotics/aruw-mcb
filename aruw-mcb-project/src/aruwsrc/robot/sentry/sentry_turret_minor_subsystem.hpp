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

#ifndef SENTRY_TURRET_MINOR_SUBSYSTEM_HPP_
#define SENTRY_TURRET_MINOR_SUBSYSTEM_HPP_
#include "tap/motor/motor_interface.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"

namespace aruwsrc::control::sentry
{
class SentryTurretMinorSubsystem final : public aruwsrc::control::turret::TurretSubsystem
{
public:
    enum class TurretID
    {
        TURRET_ID_ZERO = 0,
        TURRET_ID_ONE = 1,
    };

    SentryTurretMinorSubsystem(
        tap::Drivers& drivers,
        tap::motor::MotorInterface& pitchMotor,
        tap::motor::MotorInterface& yawMotor,
        const aruwsrc::control::turret::TurretMotorConfig& pitchMotorConfig,
        const aruwsrc::control::turret::TurretMotorConfig& yawMotorConfig,
        const aruwsrc::can::TurretMCBCanComm* turretMCB,
        TurretID turretID);

    float getMajorFrameYaw() const;
    float getMajorFramePitch() const;

    TurretID getTurretID() const { return this->turretID; };

private:
    TurretID turretID;
};

}  // namespace aruwsrc::control::sentry

#endif  // SENTRY_TURRET_MINOR_SUBSYSTEM_HPP_
