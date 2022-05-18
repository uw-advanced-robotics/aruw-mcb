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
    const tap::communication::serial::RefSerialData::Rx::MechanismID turretID,
    const uint16_t heatLimitBuffer)
    : MoveUnjamIntegralComprisedCommand(drivers, subsystem, moveIntegralCommand, unjamCommand),
      drivers(drivers),
      turretID(turretID),
      heatLimitBuffer(heatLimitBuffer)
{
}

bool RotateUnjamRefLimitedCommand::isReady()
{
    const auto &robotData = drivers.refSerial.getRobotData();

    uint16_t heat = 0;
    uint16_t heatLimit = 0;

    switch (turretID)
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
            RAISE_ERROR((&drivers), "invalid turret ID");
            // don't perform heat limiting
            heat = 0;
            heatLimit = heatLimitBuffer;
    }

    return MoveUnjamIntegralComprisedCommand::isReady() &&
           !(drivers.refSerial.getRefSerialReceivingData() && (heat + heatLimitBuffer > heatLimit));
}

bool RotateUnjamRefLimitedCommand::isFinished() const
{
    const auto &robotData = drivers.refSerial.getRobotData();

    return MoveUnjamIntegralComprisedCommand::isFinished() ||
           (drivers.refSerial.getRefSerialReceivingData() && robotData.turret.heat17ID1 != 0xffff &&
            (robotData.turret.heat17ID1 + heatLimitBuffer > robotData.turret.heatLimit17ID1));
}

}  // namespace aruwsrc::agitator
