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

#ifndef LOW_BATTERY_BUZZER_COMMAND_HPP_
#define LOW_BATTERY_BUZZER_COMMAND_HPP_

#include "tap/communication/gpio/pwm.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/drivers.hpp"

#define LOW_BATTERY_THRESHOLD 10

namespace aruwsrc::communication
{

class LowBatteryBuzzerCommand : tap::control::Command
{
public:
    LowBatteryBuzzerCommand(aruwsrc::Drivers* drivers);

    void initialize() override;

    void execute() override;

    bool isFinished() const override { return false; }

    void end(bool interrupt) override;

    const char* getName() const override { return "Low battery buzzer command"; }


private:
    Drivers* drivers;
    tap::gpio::Pwm controller;
};

}  // namespace aruwsrc::communication

#endif