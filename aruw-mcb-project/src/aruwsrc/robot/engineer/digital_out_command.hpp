/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DIGITAL_OUT_COMMAND_HPP_
#define DIGITAL_OUT_COMMAND_HPP_

#include <vector>

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/command.hpp"

namespace aruwsrc::engineer::wrist
{
class DigitalOutCommand : public tap::control::Command
{
public:
    DigitalOutCommand(tap::gpio::Digital &digital, tap::gpio::Digital::OutputPin pin, bool state)
        : digital(digital),
          pin(pin),
          state(state)
    {
    }

    void initialize() override { digital.set(pin, state); }

    void execute() override{};

    void end(bool interrupted) override{};

    bool isFinished() const override { return true; };

    const char *getName() const override { return "Digital Out Command"; }

private:
    tap::gpio::Digital &digital;
    tap::gpio::Digital::OutputPin pin;
    bool state;
};  // class WristSetpointsCommand

}  // namespace aruwsrc::engineer::wrist

#endif  // DIGITAL_OUT_COMMAND_HPP_