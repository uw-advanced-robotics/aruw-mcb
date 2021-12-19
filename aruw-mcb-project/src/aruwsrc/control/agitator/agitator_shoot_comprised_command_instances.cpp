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

#include "agitator_shoot_comprised_command_instances.hpp"

#include "agitator_subsystem.hpp"

using namespace tap::control::setpoint;

namespace aruwsrc
{
namespace agitator
{
MoveUnjamRefLimitedCommand::MoveUnjamRefLimitedCommand(
    aruwsrc::Drivers *drivers,
    AgitatorSubsystem *agitator17mm,
    float agitatorRotateAngle,
    float maxUnjamRotateAngle,
    uint32_t rotateTime,
    bool heatLimiting,
    float heatLimitBuffer)
    : tap::control::setpoint::MoveUnjamComprisedCommand(
          drivers,
          agitator17mm,
          agitatorRotateAngle,
          maxUnjamRotateAngle,
          rotateTime,
          0),
      drivers(drivers),
      heatLimiting(heatLimiting),
      heatLimitBuffer(heatLimitBuffer)
{
}

bool MoveUnjamRefLimitedCommand::isReady()
{
    const auto &robotData = drivers->refSerial.getRobotData();

    // TODO remove isOnline check when https://gitlab.com/aruw/controls/taproot/-/merge_requests/49
    // is merged in
    return MoveUnjamComprisedCommand::isReady() && setpointSubsystem->isOnline() &&
           !(drivers->refSerial.getRefSerialReceivingData() && heatLimiting &&
             (robotData.turret.heat17ID1 + heatLimitBuffer > robotData.turret.heatLimit17ID1));
}

bool MoveUnjamRefLimitedCommand::isFinished() const
{
    const auto &robotData = drivers->refSerial.getRobotData();

    return MoveUnjamComprisedCommand::isFinished() ||
           (drivers->refSerial.getRefSerialReceivingData() && heatLimiting &&
            (robotData.turret.heat17ID1 + heatLimitBuffer > robotData.turret.heatLimit17ID1));
}

}  // namespace agitator

}  // namespace aruwsrc
