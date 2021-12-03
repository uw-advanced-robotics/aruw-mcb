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

#include "aruwsrc/control/agitator/agitator_shoot_comprised_command_instances.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace agitator
{
LimitSwitchAgitatorSubsystem::LimitSwitchAgitatorSubsystem(
    aruwsrc::Drivers* drivers,
    float kp,
    float ki,
    float kd,
    float maxIAccum,
    float maxOutput,
    float agitatorGearRatio,
    tap::motor::MotorId agitatorMotorId,
    tap::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted,
    float distanceTolerance,
    uint32_t temporalTolerance,
    tap::gpio::Digital::InputPin limitSwitchPin)
    : Subsystem(drivers),
      AgitatorSubsystem(
          drivers,
          kp,
          ki,
          kd,
          maxIAccum,
          maxOutput,
          agitatorGearRatio,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted,
          true,
          distanceTolerance,
          temporalTolerance),
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
