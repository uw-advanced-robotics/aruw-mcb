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

#include "move_unjam_ref_limited_command.hpp"

#include "aruwsrc/drivers.hpp"

#include "agitator_subsystem.hpp"

using namespace tap::control::setpoint;

namespace aruwsrc
{
namespace agitator
{
MoveUnjamRefLimitedCommand::MoveUnjamRefLimitedCommand(
    aruwsrc::Drivers *drivers,
    SetpointSubsystem *setpointSubsystem,
    float moveDisplacement,
    uint32_t moveTime,
    uint32_t pauseAfterMoveTime,
    bool setToTargetOnEnd,
    float setpointTolerance,
    float unjamDisplacement,
    float unjamThreshold,
    uint32_t maxUnjamWaitTime,
    uint_fast16_t unjamCycleCount,
    bool heatLimiting,
    float heatLimitBuffer)
    : tap::control::setpoint::MoveUnjamComprisedCommand(
          drivers,
          setpointSubsystem,
          moveDisplacement,
          moveTime,
          pauseAfterMoveTime,
          setToTargetOnEnd,
          setpointTolerance,
          unjamDisplacement,
          unjamThreshold,
          maxUnjamWaitTime,
          unjamCycleCount),
      drivers(drivers),
      heatLimiting(heatLimiting),
      heatLimitBuffer(heatLimitBuffer)
{
}

bool MoveUnjamRefLimitedCommand::isReady()
{
    const auto &robotData = drivers->refSerial.getRobotData();

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
