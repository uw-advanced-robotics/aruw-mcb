/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "agitator_limiter.hpp"

#include "aruwsrc/drivers.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "aruwsrc/control/launcher/launcher_constants.hpp"

#include "../launcher/friction_wheel_subsystem.hpp"

using namespace tap::control;

namespace aruwsrc
{
namespace control::agitator
{
AgitatorLimiter::AgitatorLimiter(
    aruwsrc::Drivers* drivers,
    const tap::algorithms::SmoothPidConfig& pidParams,
    float agitatorGearRatio,
    tap::motor::MotorId agitatorMotorId,
    tap::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted,
    float jammingDistance,
    uint32_t jammingTime,
    bool jamLogicEnabled, 
    aruwsrc::control::launcher::FrictionWheelSubsystem* frictionWheelSubsystem)
    :   tap::control::Subsystem(drivers),
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
        jamChecker(this, jammingDistance, jammingTime)
{
    assert(jammingDistance >= 0);
}
    
void AgitatorLimiter::refresh()
{
    if (!agitatorIsCalibrated)
    {
        calibrateHere();
    }

    if (frictionWheelSubsystem->getCurrentLaunchSpeed() >= frictionWheelSubsystem->getDesiredLaunchSpeed() - 2) {
        agitatorRunPositionPid();
    }
    
    if (jamChecker.check())
    {
        subsystemJamStatus = true;
    }
}

}  
} 
