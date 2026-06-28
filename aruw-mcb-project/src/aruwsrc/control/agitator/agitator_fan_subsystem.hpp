/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef AGITATOR_FAN_SUBSYSTEM_HPP_
#define AGITATOR_FAN_SUBSYSTEM_HPP_

#include <cstdint>

#include "tap/communication/gpio/pwm.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::control::agitator
{
class AgitatorFanSubsystem : public tap::control::Subsystem
{
public:
    AgitatorFanSubsystem(
        tap::Drivers* drivers,
        tap::gpio::Pwm::Pin pwmPin,
        tap::gpio::Pwm::Timer pwmTimer,
        uint32_t pwmFrequencyHz);

    void initialize() override;

    void refreshSafeDisconnect() override;

    void setFanDuty(float duty);

    const char* getName() const override { return "agitator fan"; }

private:
    tap::gpio::Pwm::Pin pwmPin;
    tap::gpio::Pwm::Timer pwmTimer;
    uint32_t pwmFrequencyHz;
};  // class AgitatorFanSubsystem

}  // namespace aruwsrc::control::agitator

#endif  // AGITATOR_FAN_SUBSYSTEM_HPP_
