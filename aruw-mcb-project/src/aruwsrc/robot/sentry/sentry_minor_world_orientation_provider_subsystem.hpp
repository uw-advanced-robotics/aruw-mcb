/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef SENTRY_MINOR_WORLD_PROVIDER_SUBSYSTEM_HPP_
#define SENTRY_MINOR_WORLD_PROVIDER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"

namespace aruwsrc::control::turret
{

class SentryMinorWorldYawProviderSubsystem : public tap::control::Subsystem
{
public:
    SentryMinorWorldYawProviderSubsystem(
        TurretMotor turretMotor,
        const aruwsrc::can::TurretMCBCanComm& turretMCB,
        tap::Drivers* drivers);

    void initialize() override;

    void refresh() override;

    float getYaw() const;

    float getYawVel() const;

    float getPitch() const;

    float getPitchVel() const;

    inline void refreshSafeDisconnect() override{};

    const char* getName() const override { return "Sentry Minor World Yaw Provider Subsystem"; }
};  // class SentryMinorWorldYawProviderSubsystem

}  // namespace aruwsrc::control::turret
#endif  // SENTRY_MINOR_WORLD_PROVIDER_SUBSYSTEM_HPP_
