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

#ifndef TURRET_MCB_HOPPER_COVER_SUBSYSTEM_HPP_
#define TURRET_MCB_HOPPER_COVER_SUBSYSTEM_HPP_

#include "tap/control/command_scheduler.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/servo.hpp"
#include "tap/util_macros.hpp"

#include "modm/math/filter/pid.hpp"

namespace aruwsrc::control
{
class TurretMCBHopperSubsystem : public tap::control::Subsystem
{
public:
    TurretMCBHopperSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
          drivers(drivers)
    {
    }

    mockable void setOpen() { drivers->turretMCBCanComm.setOpenHopperCover(true); }

    mockable void setClose() { drivers->turretMCBCanComm.setOpenHopperCover(false); }

    void refresh() override {}

    void runHardwareTests() override {}

    void onHardwareTestStart() override {}

    void onHardwareTestComplete() override {}

    const char *getName() override { return "turret MCB hopper"; }

private:
    tap::Drivers *drivers;
};

}  // namespace aruwsrc::control

#endif  // TURRET_MCB_HOPPER_COVER_SUBSYSTEM_HPP_
