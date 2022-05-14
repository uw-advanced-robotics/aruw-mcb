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

#include "launch_speed_gated_agitator.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/motor/dji_motor.hpp"

#include "../launcher/friction_wheel_subsystem.hpp"
#include "aruwsrc/control/launcher/launcher_constants.hpp"
#include "aruwsrc/drivers.hpp"

using namespace tap::control;


namespace aruwsrc::control::agitator
{
LaunchSpeedGatedAgitator::LaunchSpeedGatedAgitator(
    aruwsrc::Drivers* drivers,
    const tap::algorithms::SmoothPidConfig& pidParams,
    float agitatorGearRatio,
    tap::motor::MotorId agitatorMotorId,
    tap::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted,
    float jammingDistance,
    uint32_t jammingTime,
    bool jamLogicEnabled,
    const aruwsrc::control::launcher::FrictionWheelSubsystem* frictionWheelSubsystem,
    float launchSpeedThreshold)
    : tap::control::Subsystem(drivers),
      aruwsrc::agitator::AgitatorSubsystem(
          drivers,
          pidParams,
          agitatorGearRatio,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted,
          jammingDistance,
          jammingTime,
          jamLogicEnabled),
      frictionWheelSubsystem(frictionWheelSubsystem),
      launchSpeedThreshold(launchSpeedThreshold)
{
    assert(jammingDistance >= 0);
}

void LaunchSpeedGatedAgitator::refresh()
{
    AgitatorSubsystem::refresh();

    if (frictionWheelSubsystem->getCurrentLaunchSpeed() <=
        launchSpeedThreshold)
    {
        agitatorMotor.setDesiredOutput(0);
    }

}

}  // namespace aruwsrc::control::agitator
