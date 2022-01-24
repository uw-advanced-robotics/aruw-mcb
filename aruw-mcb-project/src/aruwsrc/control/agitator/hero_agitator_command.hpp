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

#ifndef HERO_AGITATOR_COMMAND_HPP_
#define HERO_AGITATOR_COMMAND_HPP_

#include "tap/control/comprised_command.hpp"
#include "tap/control/setpoint/commands/move_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"

using tap::gpio::Digital;

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
class AgitatorSubsystem;

/**
 * A command that launches a projectile for the Hero firing subsystem.
 * First the kicker and waterwheel are rotated to fire a single shot,
 * then the kicker and waterwheel are rotated to load the next ball.
 */
class HeroAgitatorCommand : public tap::control::ComprisedCommand
{
public:
    HeroAgitatorCommand(
        aruwsrc::Drivers* drivers,
        AgitatorSubsystem* kickerAgitator,
        AgitatorSubsystem* waterwheelAgitator,
        float kickerShootRotateAngle,
        float kickerShootRotateTime,
        float kickerLoadRotateAngle,
        float waterwheelLoadRotateAngle,
        uint32_t loadRotateTime,
        float waterwheelMaxUnjamAngle,
        bool heatLimiting,
        uint16_t heatLimitBuffer);

    bool isReady() override;

    bool isFinished() const override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    const char* getName() const override { return "hero agitator shoot"; }

private:
    tap::control::setpoint::MoveCommand kickerFireCommand;
    tap::control::setpoint::MoveCommand kickerLoadCommand;
    tap::control::setpoint::MoveUnjamComprisedCommand waterwheelLoadCommand;

    enum HeroAgitatorState
    {
        SHOOTING,
        LOAD,
        FINISHED
    };

    aruwsrc::Drivers* drivers;
    AgitatorSubsystem* kickerAgitator;
    AgitatorSubsystem* waterwheelAgitator;
    HeroAgitatorState currState;
    bool heatLimiting;
    uint16_t heatLimitBuffer;
    uint16_t startingHeat;

    const tap::gpio::Digital::InputPin LIMIT_SWITCH_PIN = Digital::InputPin::A;

    void beginLoading();
};  // class HeroAgitatorCommand

}  // namespace aruwsrc::agitator

#endif  // HERO_AGITATOR_COMMAND_HPP_
