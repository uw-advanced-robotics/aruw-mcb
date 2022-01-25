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
 * A command that launches a projectile and loads a new ball into the kicker.
 */
class HeroAgitatorCommand : public tap::control::ComprisedCommand
{
public:
    /**
     * Constructs a HeroAgitatorCommand with the given agitator motors, shooting
     * rotation and time, loading rotation and time, and heat limiting params.
     */
    HeroAgitatorCommand(
        aruwsrc::Drivers* drivers,
        AgitatorSubsystem* kickerAgitator,
        AgitatorSubsystem* waterwheelAgitator,
        float kickerShootRotateAngle,
        uint32_t kickerShootRotateTime,
        float kickerLoadRotateAngle,
        float waterwheelLoadRotateAngle,
        uint32_t loadRotateTime,
        float waterwheelMaxUnjamAngle,
        bool heatLimiting,
        uint16_t heatLimitBuffer);

    /**
     * @return Whether the agitator subsystems are online and heat is below
     * the heat limit buffer, indicating that the command is ready.
     */
    bool isReady() override;

    /**
     * @return Whether the command has finished running.
     */
    bool isFinished() const override;

    /**
     * Initializes the command. Spins the kicker to shoot the ball if the
     * limit switch is triggered. Spins the agitator and kicker to load a ball
     * otherwise.
     */
    void initialize() override;

    /**
     * Schedules commands to load a ball if the kicker has finished shooting a
     * ball or if the load commands have finished but the limit switch isn't pressed.
     * Enters the finished state if the limit switch is pressed.
     */
    void execute() override;

    /**
     * Removes all commands from scheduler, resets command to initial state.
     *
     * @param interrupted Whether the command was ended due to an interruption.
     */
    void end(bool interrupted) override;

    /**
     * @return Name of command
     */
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

    /**
     * Enters the loading state and schedules commands to rotate the kicker
     * and agitator.
     */
    void beginLoading();
};  // class HeroAgitatorCommand

}  // namespace aruwsrc::agitator

#endif  // HERO_AGITATOR_COMMAND_HPP_
