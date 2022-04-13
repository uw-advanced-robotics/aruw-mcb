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

#include "stepper_motor.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/can/can_rx_listener.hpp"
#include "tap/drivers.hpp"

#include "motor_interface.hpp"

tap::gpio::Digital::OutputPin direction = tap::gpio::Digital::OutputPin::E;
tap::gpio::Pwm::Pin pulse = tap::gpio::Pwm::Pin::X;

namespace tap
{
namespace motor
{
static constexpr uint16_t ENC_RESOLUTION = 1600;

StepperMotor::StepperMotor(Drivers* drivers, bool isInverted, const char* name) : drivers(drivers)
{
}

void StepperMotor::initialize()
{
    drivers->pwm.setTimerFrequency(tap::gpio::Pwm::Timer::TIMER8, 100);
}

void StepperMotor::setDesiredOutput(int32_t desiredOutput)
{
    int16_t desOutputNotInverted =
        static_cast<int16_t>(tap::algorithms::limitVal<int32_t>(desiredOutput, SHRT_MIN, SHRT_MAX));
    this->desiredOutput = motorInverted ? -desOutputNotInverted : desOutputNotInverted;
    drivers->pwm.write(desiredOutput, pulse);
    if (desiredOutput >= 0)
    {
        drivers->digital.set(direction, false);
    }
    else
    {
        drivers->digital.set(direction, true);
    }
}

bool StepperMotor::isMotorOnline() const { return true; };

int16_t StepperMotor::getOutputDesired() const { return desiredOutput; }

int8_t StepperMotor::getTemperature() const { return 0; }

int16_t StepperMotor::getTorque() const { return 0; };

int16_t StepperMotor::getShaftRPM() const { return 0; };
}  // namespace motor
}  // namespace tap