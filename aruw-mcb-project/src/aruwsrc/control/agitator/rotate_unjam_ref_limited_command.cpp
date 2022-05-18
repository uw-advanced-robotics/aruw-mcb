/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "rotate_unjam_ref_limited_command.hpp"

#include "tap/errors/create_errors.hpp"

#include "aruwsrc/drivers.hpp"

#include "agitator_subsystem.hpp"

using namespace tap::communication::serial;
using namespace tap::control::setpoint;

namespace aruwsrc::agitator
{
RotateUnjamRefLimitedCommand::RotateUnjamRefLimitedCommand(
    aruwsrc::Drivers &drivers,
    IntegrableSetpointSubsystem &subsystem,
    tap::control::setpoint::MoveIntegralCommand &moveIntegralCommand,
    tap::control::setpoint::UnjamIntegralCommand &unjamCommand,
    const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID,
    const uint16_t heatLimitBuffer)
    : MoveUnjamIntegralComprisedCommand(drivers, subsystem, moveIntegralCommand, unjamCommand),
      drivers(drivers),
      firingSystemMechanismID(firingSystemMechanismID),
      heatLimitBuffer(heatLimitBuffer)
{
}

bool RotateUnjamRefLimitedCommand::isReady()
{
    return MoveUnjamIntegralComprisedCommand::isReady() && enoughHeatToLaunchProjectile();
}

bool RotateUnjamRefLimitedCommand::isFinished() const
{
    return MoveUnjamIntegralComprisedCommand::isFinished() || !enoughHeatToLaunchProjectile();
}

bool RotateUnjamRefLimitedCommand::enoughHeatToLaunchProjectile() const
{
    if (!drivers.refSerial.getRefSerialReceivingData())
    {
        return true;
    }

    const auto &robotData = drivers.refSerial.getRobotData();

    uint16_t heat = 0, heatLimit = 0;

    switch (firingSystemMechanismID)
    {
        case RefSerialData::Rx::MechanismID::TURRET_17MM_1:
            heat = robotData.turret.heat17ID1;
            heatLimit = robotData.turret.heatLimit17ID1;
            break;
        case RefSerialData::Rx::MechanismID::TURRET_17MM_2:
            heat = robotData.turret.heat17ID2;
            heatLimit = robotData.turret.heatLimit17ID2;
            break;
        case RefSerialData::Rx::MechanismID::TURRET_42MM:
            heat = robotData.turret.heat42;
            heatLimit = robotData.turret.heatLimit42;
            break;
        default:
            RAISE_ERROR((&drivers), "invalid barrel mechanism ID");
            // don't perform heat limiting
            heat = 0;
            heatLimit = heatLimitBuffer;
    }

    const bool heatBelowLimit = heat + heatLimitBuffer <= heatLimit;

    return !RefSerial::heatAndLimitValid(heat, heatLimit) || heatBelowLimit;
}

}  // namespace aruwsrc::agitator
