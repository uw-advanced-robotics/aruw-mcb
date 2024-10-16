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

#include "hopper_subsystem.hpp"

#include "tap/architecture/clock.hpp"

namespace aruwsrc
{
namespace control
{
HopperSubsystem::HopperSubsystem(
    tap::Drivers* drivers,
    tap::gpio::Pwm::Pin pwmPin,
    float open,
    float close,
    float pwmRampSpeed)
    : tap::control::Subsystem(drivers),
      hopper(drivers, pwmPin, open, close, pwmRampSpeed),
      hopperTestCommand(this)
{
    this->setTestCommand(&hopperTestCommand);
    hopper.setTargetPwm(close);
}

void HopperSubsystem::setOpen() { hopper.setTargetPwm(hopper.getMaxPWM()); }

void HopperSubsystem::setClose() { hopper.setTargetPwm(hopper.getMinPWM()); }

void HopperSubsystem::refresh() { hopper.updateSendPwmRamp(); }

float HopperSubsystem::getOpenPWM() { return hopper.getMaxPWM(); }

float HopperSubsystem::getClosePWM() { return hopper.getMinPWM(); }

HopperTestCommand::HopperTestCommand(HopperSubsystem* subsystem) : subsystem(subsystem)
{
    this->addSubsystemRequirement(subsystem);
}

void HopperTestCommand::initialize()
{
    this->subsystem->setOpen();
    this->startTime = tap::arch::clock::getTimeMilliseconds();
}

void HopperTestCommand::end(bool) { this->subsystem->setClose(); }

bool HopperTestCommand::isFinished() const
{
    return tap::arch::clock::getTimeMilliseconds() - this->startTime > 1000;
}
}  // namespace control

}  // namespace aruwsrc
