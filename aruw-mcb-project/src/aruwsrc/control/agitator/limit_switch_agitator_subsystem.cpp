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

#include "limit_switch_agitator_subsystem.hpp"

#include "aruwlib/drivers.hpp"

#include "aruwsrc/control/agitator/agitator_shoot_comprised_command_instances.hpp"

using namespace aruwlib::algorithms;

namespace aruwsrc
{
namespace agitator
{
LimitSwitchAgitatorSubsystem::LimitSwitchAgitatorSubsystem(
    aruwlib::Drivers* drivers,
    const aruwlib::algorithms::PidConfigStruct& pidConfig,
    float agitatorGearRatio,
    aruwlib::motor::MotorId agitatorMotorId,
    aruwlib::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted,
    float jamDistanceTolerance,
    uint32_t jamTemporalTolerance,
    aruwlib::gpio::Digital::InputPin limitSwitchPin)
    : Subsystem(drivers),
      AgitatorSubsystem(
          drivers,
          pidConfig,
          agitatorGearRatio,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted,
          true,
          jamDistanceTolerance,
          jamTemporalTolerance),
      limitSwitchPin(limitSwitchPin),
      digital(&drivers->digital),
      ballsInTube(0),
      prevInitializeCount(0)
{
}

void LimitSwitchAgitatorSubsystem::refresh()
{
    AgitatorSubsystem::refresh();

    // Ball has been removed from between waterwheel and kicker
    if (agitator::ShootCommand42mm::getInitializeCount() != prevInitializeCount)
    {
        ballsInTube = std::max(0, ballsInTube - 1);
        prevInitializeCount = agitator::ShootCommand42mm::getInitializeCount();
    }

    // Limit switch rising edge
    const bool newLimitSwitchPressed = !digital->read(limitSwitchPin);
    if (newLimitSwitchPressed && !limitSwitchPressed)
    {
        ballsInTube++;
    }
    limitSwitchPressed = newLimitSwitchPressed;
}

}  // namespace agitator

}  // namespace aruwsrc
