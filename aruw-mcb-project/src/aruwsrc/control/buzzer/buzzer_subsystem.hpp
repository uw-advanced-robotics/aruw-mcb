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

#ifndef BUZZER_SUBSYSTEM_HPP_
#define BUZZER_SUBSYSTEM_HPP_

#include "tap/communication/gpio/pwm.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::control::buzzer
{
class BuzzerSubsystem : public tap::control::Subsystem
{
public:
    BuzzerSubsystem(tap::Drivers* drivers);

    const char* getName() const override { return "Buzzer"; }

    void playNoise();

    void refreshSafeDisconnect() override { stop(); }

    void stop();
};

}  // namespace aruwsrc::control::buzzer

#endif  // BUZZER_SUBSYSTEM_HPP_