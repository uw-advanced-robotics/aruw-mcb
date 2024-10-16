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
#include "tap/motor/servo.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "modm/math/filter/pid.hpp"

namespace aruwsrc::control
{
/**
 * Subsystem that wraps a turret hopper cover that is controlled via the
 * `TurretMCBCanComm` class. By default, the turret hopper is closed. A command
 * may tell the hopper to be opened or closed.
 */
class TurretMCBHopperSubsystem : public tap::control::Subsystem
{
public:
    TurretMCBHopperSubsystem(
        tap::Drivers *drivers,
        aruwsrc::can::TurretMCBCanComm &turretMCBCanComm)
        : tap::control::Subsystem(drivers),
          turretMCBCanComm(turretMCBCanComm)
    {
    }

    mockable void setOpen()
    {
        hopperOpen = true;
        turretMCBCanComm.setOpenHopperCover(true);
    }

    mockable void setClose()
    {
        hopperOpen = false;
        turretMCBCanComm.setOpenHopperCover(false);
    }

    void refresh() override {}

    const char *getName() const override { return "turret MCB hopper"; }

    bool getIsHopperOpen() const { return hopperOpen; }

private:
    aruwsrc::can::TurretMCBCanComm &turretMCBCanComm;

    bool hopperOpen = false;
};

}  // namespace aruwsrc::control

#endif  // TURRET_MCB_HOPPER_COVER_SUBSYSTEM_HPP_
