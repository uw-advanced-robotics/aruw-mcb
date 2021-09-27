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

#include "snail_friction_wheel_subsystem.hpp"

#include "tap/drivers.hpp"

namespace aruwsrc::launcher
{
SnailFrictionWheelSubsystem::SnailFrictionWheelSubsystem(
    tap::Drivers *drivers,
    tap::gpio::Pwm::Pin leftFlywheelPin,
    tap::gpio::Pwm::Pin rightFlywheelPin)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      leftFlywheelPin(leftFlywheelPin),
      rightFlywheelPin(rightFlywheelPin),
      targetPwm(),
      initialized(false),
      initializeTimer()
{
}

void SnailFrictionWheelSubsystem::initialize()
{
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::TIMER8, 200);
}

void SnailFrictionWheelSubsystem::refresh() { runProtothread(); }

void SnailFrictionWheelSubsystem::runHardwareTests() {}

void SnailFrictionWheelSubsystem::onHardwareTestStart() {}

void SnailFrictionWheelSubsystem::onHardwareTestComplete() {}

bool SnailFrictionWheelSubsystem::runProtothread()
{
    PT_BEGIN();

    // Wait until ref system is offline or we are told the shooter
    // has power.
    PT_WAIT_UNTIL(
        !drivers->refSerial.getRefSerialReceivingData() ||
        drivers->refSerial.getRobotData().shooterHasPower);

    // Init the flywheels and wait for initialization sequence to finish
    PT_CALL(initFlywheels());

    while (true)
    {
        if (drivers->refSerial.getRefSerialReceivingData() &&
            !drivers->refSerial.getRobotData().shooterHasPower)
        {
            initialized = false;
        }

        if (initialized)
        {
            targetPwm.update(FRICTION_WHEEL_PWM_PERCENT_INCR);
            drivers->pwm.write(targetPwm.getValue(), leftFlywheelPin);
            drivers->pwm.write(targetPwm.getValue(), rightFlywheelPin);
        }
        else
        {
            PT_CALL(initFlywheels());
        }

        PT_YIELD();
    }

    PT_END();
}

modm::ResumableResult<bool> SnailFrictionWheelSubsystem::initFlywheels()
{
    RF_BEGIN(0);

    targetPwm.reset(ZERO_CYCLE);

    initializeTimer.restart(FRICTION_WHEEL_INIT_WAIT);

    drivers->pwm.write(targetPwm.getValue(), leftFlywheelPin);
    drivers->pwm.write(targetPwm.getValue(), rightFlywheelPin);

    PT_WAIT_UNTIL(initializeTimer.execute());

    initialized = true;

    RF_END();
}

void SnailFrictionWheelSubsystem::startFrictionWheels() { targetPwm.setTarget(SPEED_15M_PER_SEC); }

void SnailFrictionWheelSubsystem::stopFrictionWheels() { initialized = false; }

}  // namespace aruwsrc::launcher
