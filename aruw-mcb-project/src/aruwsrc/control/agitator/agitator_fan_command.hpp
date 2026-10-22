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

#ifndef AGITATOR_FAN_COMMAND_HPP_
#define AGITATOR_FAN_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "agitator_fan_subsystem.hpp"

namespace aruwsrc::control::agitator
{
class AgitatorFanCommand : public tap::control::Command
{
public:
    AgitatorFanCommand(
        tap::Drivers* drivers,
        AgitatorFanSubsystem& fan,
        float onDuty = 1.0f,
        float offDuty = 0.0f,
        const tap::control::Command* disableWhenScheduled = nullptr);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override { return false; }

    const char* getName() const override { return "agitator fan"; }

private:
    void updateFanDuty();

    bool shouldRunFan() const;

    void setFanDuty(float duty);

    tap::Drivers* drivers;
    AgitatorFanSubsystem& fan;
    float onDuty;
    float offDuty;
    const tap::control::Command* disableWhenScheduled;
};  // class AgitatorFanCommand

}  // namespace aruwsrc::control::agitator

#endif  // AGITATOR_FAN_COMMAND_HPP_
