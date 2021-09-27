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

#ifndef SNAIL_FRICTION_WHEEL_SUBSYSTEM_HPP_
#define SNAIL_FRICTION_WHEEL_SUBSYSTEM_HPP_

#include "tap/algorithms/ramp.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"
#include "tap/architecture/timeout.hpp"
#include "modm/processing.hpp"

namespace aruwsrc::launcher
{
class SnailFrictionWheelSubsystem : public tap::control::Subsystem,
                                    public ::modm::pt::Protothread,
                                    modm::Resumable<1>
{
public:
    SnailFrictionWheelSubsystem(
        tap::Drivers *drivers,
        tap::gpio::Pwm::Pin leftFlywheelPin,
        tap::gpio::Pwm::Pin rightFlywheelPin);

    void initialize() override;

    void refresh() override;

    void runHardwareTests() override;

    void onHardwareTestStart() override;

    void onHardwareTestComplete() override;

    const char *getName() override { return "old soldier friction wheel"; }

    void startFrictionWheels();

    void stopFrictionWheels();

private:
    static constexpr uint32_t FRICTION_WHEEL_INIT_WAIT = 2000;

    static constexpr float ZERO_CYCLE = 0.1f;
    static constexpr float SPEED_15M_PER_SEC = 0.3f;
    static constexpr float FRICTION_WHEEL_PWM_PERCENT_INCR = 0.01f;

    tap::Drivers *drivers;
    tap::gpio::Pwm::Pin leftFlywheelPin, rightFlywheelPin;
    tap::algorithms::Ramp targetPwm;
    bool initialized;
    tap::arch::MilliTimeout initializeTimer;

    bool runProtothread();

    modm::ResumableResult<bool> initFlywheels();
};

}  // namespace aruwsrc::launcher

#endif  // SNAIL_FRICTION_WHEEL_SUBSYSTEM_HPP_
