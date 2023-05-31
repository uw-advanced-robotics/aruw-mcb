/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "barrel_switcher_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::control
{
BarrelSwitcherSubsystem::BarrelSwitcherSubsystem(
    tap::Drivers* drivers,
    aruwsrc::control::StallThresholdConfig config,
    tap::motor::MotorId motorid)
    : Subsystem(drivers),
      config(config),
      motor(drivers, motorid, tap::can::CanBus::CAN_BUS1, false, "barrel switching motor")
{
}

void BarrelSwitcherSubsystem::initialize()
{
    barrelState = BarrelState::IDLE;
    motor.initialize();
}

void BarrelSwitcherSubsystem::refresh()
{
    outputDesiredDebug = motor.getOutputDesired();
    torqueDebug = motor.getTorque();
    shaftRPMDebug = motor.getShaftRPM();
    stalled = this->isStalled();

    switch (barrelState)
    {
        case BarrelState::USING_LEFT_BARREL:
            if (!isStalled() && !inPosition)
            {
                setMotorOutput(MOTOR_OUTPUT);
            }
            else
            {
                inPosition = true;
                setMotorOutput(0);
            }
            break;
        case BarrelState::USING_RIGHT_BARREL:
            if (!isStalled() && !inPosition)
            {
                setMotorOutput(-MOTOR_OUTPUT);
            }
            else
            {
                inPosition = true;
                setMotorOutput(0);
            }
            break;
        case BarrelState::IDLE:
            setMotorOutput(0);
            break;
    }
}

BarrelState BarrelSwitcherSubsystem::getBarrelState() const { return barrelState; }

void BarrelSwitcherSubsystem::setMotorOutput(int32_t desiredOutput)
{
    motor.setDesiredOutput(desiredOutput);
}

bool BarrelSwitcherSubsystem::isStalled() const
{
    return (
        (fabs(motor.getShaftRPM()) < config.maxRPM) &&
        (fabsl(motor.getTorque()) > config.minTorque));
}

bool BarrelSwitcherSubsystem::isInPosition() const { return inPosition; }

void BarrelSwitcherSubsystem::useRight()
{
    barrelState = BarrelState::USING_RIGHT_BARREL;
    inPosition = false;
}

void BarrelSwitcherSubsystem::useLeft()
{
    barrelState = BarrelState::USING_LEFT_BARREL;
    inPosition = false;
}

void BarrelSwitcherSubsystem::stop()
{
    this->setMotorOutput(0);
    barrelState = BarrelState::IDLE;
}
};  // namespace aruwsrc::control