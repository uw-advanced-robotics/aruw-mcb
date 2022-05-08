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

#include "aruwsrc/drivers.hpp"

#include "agitator_subsystem.hpp"

using namespace tap::control::setpoint;

namespace aruwsrc::agitator
{
RotateUnjamRefLimitedCommand::RotateUnjamRefLimitedCommand(
    aruwsrc::Drivers &drivers,
    IntegrableSetpointSubsystem &subsystem,
    tap::control::setpoint::MoveIntegralCommand &moveIntegralCommand,
    tap::control::setpoint::UnjamIntegralCommand &unjamCommand,
    uint16_t heatLimitBuffer)
    : MoveUnjamIntegralComprisedCommand(drivers, subsystem, moveIntegralCommand, unjamCommand),
      drivers(drivers),
      heatLimitBuffer(heatLimitBuffer)
{
}

bool RotateUnjamRefLimitedCommand::isReady()
{
    const auto &robotData = drivers.refSerial.getRobotData();

    return MoveUnjamIntegralComprisedCommand::isReady() &&
           !(drivers.refSerial.getRefSerialReceivingData() &&
             (robotData.turret.heat17ID1 != 0xffff &&
              (robotData.turret.heat17ID1 + heatLimitBuffer > robotData.turret.heatLimit17ID1)));
}

bool RotateUnjamRefLimitedCommand::isFinished() const
{
    const auto &robotData = drivers.refSerial.getRobotData();

    return MoveUnjamIntegralComprisedCommand::isFinished() ||
           (drivers.refSerial.getRefSerialReceivingData() && robotData.turret.heat17ID1 != 0xffff &&
            (robotData.turret.heat17ID1 + heatLimitBuffer > robotData.turret.heatLimit17ID1));
}

}  // namespace aruwsrc::agitator
